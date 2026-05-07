#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "radar_msgs/msg/radar_target.hpp"
#include "radar_msgs/msg/radar_target_array.hpp"

#include <cmath>
#include <vector>
#include <algorithm>
#include <queue>
#include <random>
#include <map>
#include <limits>

using namespace std::placeholders;

// -------------------- 简单卡尔曼滤波器 --------------------
class KalmanFilter {
public:
    KalmanFilter(float q = 0.01f, float r = 0.1f) : q_(q), r_meas_(r), initialized_(false) {}

    void init(float x, float y, float rad) {
        x_ = x; y_ = y; r_ = rad;
        p_x_ = 1.0f; p_y_ = 1.0f; p_r_ = 1.0f;
        initialized_ = true;
    }

    void update(float meas_x, float meas_y, float meas_r) {
        if (!initialized_) {
            init(meas_x, meas_y, meas_r);
            return;
        }
        p_x_ += q_;
        p_y_ += q_;
        p_r_ += q_;
        float k_x = p_x_ / (p_x_ + r_meas_);
        float k_y = p_y_ / (p_y_ + r_meas_);
        float k_r = p_r_ / (p_r_ + r_meas_);
        x_ = x_ + k_x * (meas_x - x_);
        y_ = y_ + k_y * (meas_y - y_);
        r_ = r_ + k_r * (meas_r - r_);
        p_x_ = (1.0f - k_x) * p_x_;
        p_y_ = (1.0f - k_y) * p_y_;
        p_r_ = (1.0f - k_r) * p_r_;
    }

    float getX() const { return x_; }
    float getY() const { return y_; }
    float getR() const { return r_; }
    bool isInitialized() const { return initialized_; }

private:
    float x_ = 0, y_ = 0, r_ = 0;
    float p_x_ = 1, p_y_ = 1, p_r_ = 1;
    float q_, r_meas_;
    bool initialized_ = false;
};

struct TrackedTarget {
    int id;
    KalmanFilter kf;
    int missed_count = 0;
    int confirmed_count = 0;
    bool matched_this_frame = false;
};

class RadarNode : public rclcpp::Node
{
public:
    RadarNode() : Node("radar_node"), next_id_(1)
    {
        RCLCPP_INFO(this->get_logger(), "雷达花盆检测节点 (自动调参 + 卡尔曼跟踪)");

        // ---------- 基础参数 ----------
        this->declare_parameter<double>("min_range", 0.10);
        this->declare_parameter<double>("max_range", 1.5);
        this->declare_parameter<double>("angle_min", 20 * M_PI/180);
        this->declare_parameter<double>("angle_max", 140 * M_PI/180);
        this->declare_parameter<int>("median_window", 5);

        // ---------- 自动调参开关 ----------
        this->declare_parameter<bool>("auto_tune", true);   // 是否开启自动调参
        // 自动调参分段阈值 (米)
        this->declare_parameter<double>("range_1", 0.3);   // 小于此值为近处
        this->declare_parameter<double>("range_2", 0.4);    // 大于此值为远处，中间为中距
        this->declare_parameter<double>("range_3", 0.7);    // 大于此值为远处，中间为中距
        this->declare_parameter<double>("range_4", 1.0);
        this->declare_parameter<double>("range_5", 0.5);
        this->declare_parameter<double>("range_6", 0.5);
        this->declare_parameter<double>("range_7", 0.5);

        // ---------- 以下参数在 auto_tune=true 时会根据距离自动调整，否则使用手动设定值 ----------
        // DBSCAN 参数
        this->declare_parameter<double>("dbscan_eps", 0.05);
        this->declare_parameter<int>("dbscan_min_pts", 5);
        // RANSAC 参数
        this->declare_parameter<int>("ransac_iter", 200);
        this->declare_parameter<double>("ransac_thresh", 0.01);
        // 半径过滤
        this->declare_parameter<double>("final_min_radius", 0.07);
        this->declare_parameter<double>("final_max_radius", 0.10);
        // 圆形置信度
        this->declare_parameter<double>("max_mean_error", 0.01);
        this->declare_parameter<double>("min_angle_span", 0.4);
        // 墙角过滤 (默认关闭)
        this->declare_parameter<bool>("enable_corner_filter", false);
        this->declare_parameter<double>("wall_eigenvalue_ratio", 0.10);
        this->declare_parameter<double>("min_angle_coverage", 120.0 * M_PI / 180.0);
        this->declare_parameter<double>("max_radius_stddev", 0.03);
        // 跟踪参数
        this->declare_parameter<double>("match_distance", 0.3);
        this->declare_parameter<int>("confirm_frames", 2);
        this->declare_parameter<int>("max_missed", 15);
        this->declare_parameter<double>("kalman_q", 0.005);
        this->declare_parameter<double>("kalman_r", 0.1);

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&RadarNode::lidar_callback, this, _1));
        target_pub_ = this->create_publisher<radar_msgs::msg::RadarTargetArray>("radar_targets", 10);
    }

private:
    struct Circle { float cx, cy, r; };

    // 中值滤波 (不变)
    std::vector<float> medianFilter(const std::vector<float>& data, int window_size)
    {
        if (data.empty() || window_size % 2 == 0) return data;
        int half = window_size / 2;
        std::vector<float> filtered(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            std::vector<float> window;
            for (int j = -half; j <= half; ++j) {
                int idx = static_cast<int>(i) + j;
                if (idx >= 0 && idx < static_cast<int>(data.size()))
                    window.push_back(data[idx]);
            }
            if (!window.empty()) {
                std::sort(window.begin(), window.end());
                filtered[i] = window[window.size() / 2];
            } else {
                filtered[i] = data[i];
            }
        }
        return filtered;
    }

    // 点云转换 (不变)
    std::vector<std::pair<float, float>> convertToPoints(
        const sensor_msgs::msg::LaserScan::SharedPtr msg,
        const std::vector<float>& ranges_filtered)
    {
        std::vector<std::pair<float, float>> points;
        double min_range = this->get_parameter("min_range").as_double();
        double max_range = this->get_parameter("max_range").as_double();
        double angle_min = this->get_parameter("angle_min").as_double();
        double angle_max = this->get_parameter("angle_max").as_double();

        float angle = msg->angle_min;
        for (size_t i = 0; i < ranges_filtered.size(); ++i) {
            float range = ranges_filtered[i];
            if (range > min_range && range < max_range && std::isfinite(range)) {
                if (angle > angle_min && angle < angle_max) {
                    float x = range * cos(angle);
                    float y = range * sin(angle);
                    points.emplace_back(x, y);
                }
            }
            angle += msg->angle_increment;
        }
        return points;
    }

    // DBSCAN 聚类 (不变)
    std::vector<std::vector<std::pair<float, float>>> dbscanClustering(
        const std::vector<std::pair<float, float>>& points,
        double eps, int min_pts)
    {
        std::vector<std::vector<std::pair<float, float>>> clusters;
        if (points.empty()) return clusters;

        const size_t N = points.size();
        std::vector<bool> visited(N, false);
        std::vector<int> labels(N, -1);

        size_t cluster_id = 0;
        for (size_t i = 0; i < N; ++i) {
            if (visited[i]) continue;
            visited[i] = true;
            std::vector<size_t> neighbors;
            for (size_t j = 0; j < N; ++j) {
                if (i == j) continue;
                double dx = points[i].first - points[j].first;
                double dy = points[i].second - points[j].second;
                if (std::hypot(dx, dy) < eps) neighbors.push_back(j);
            }
            if (neighbors.size() + 1 < static_cast<size_t>(min_pts)) {
                labels[i] = -2;
                continue;
            }
            clusters.emplace_back();
            std::queue<size_t> q;
            q.push(i);
            labels[i] = cluster_id;
            clusters.back().push_back(points[i]);
            while (!q.empty()) {
                size_t idx = q.front(); q.pop();
                std::vector<size_t> nb2;
                for (size_t j = 0; j < N; ++j) {
                    if (idx == j) continue;
                    double dx = points[idx].first - points[j].first;
                    double dy = points[idx].second - points[j].second;
                    if (std::hypot(dx, dy) < eps) nb2.push_back(j);
                }
                if (nb2.size() + 1 >= static_cast<size_t>(min_pts)) {
                    for (size_t nb : nb2) {
                        if (!visited[nb]) { visited[nb] = true; q.push(nb); }
                        if (labels[nb] == -1 || labels[nb] == -2) {
                            labels[nb] = cluster_id;
                            clusters.back().push_back(points[nb]);
                        }
                    }
                } else {
                    for (size_t nb : nb2) {
                        if (labels[nb] == -1 || labels[nb] == -2) {
                            labels[nb] = cluster_id;
                            clusters.back().push_back(points[nb]);
                        }
                    }
                }
            }
            ++cluster_id;
        }
        return clusters;
    }

    // 三点定圆 (不变)
    Circle circleFromThreePoints(const std::pair<float, float>& p1,
                                 const std::pair<float, float>& p2,
                                 const std::pair<float, float>& p3)
    {
        float x1 = p1.first, y1 = p1.second;
        float x2 = p2.first, y2 = p2.second;
        float x3 = p3.first, y3 = p3.second;
        float denom = (x2 - x1)*(y3 - y2) - (y2 - y1)*(x3 - x2);
        if (std::abs(denom) < 1e-6) return {0,0,0};
        float ma = (y2 - y1) / (x2 - x1);
        float mb = (y3 - y2) / (x3 - x2);
        float cx = (ma*mb*(y1 - y3) + mb*(x1 + x2) - ma*(x2 + x3)) / (2 * (mb - ma));
        float cy = -1/ma * (cx - (x1 + x2)/2) + (y1 + y2)/2;
        float r = std::hypot(x1 - cx, y1 - cy);
        return {cx, cy, r};
    }

    // RANSAC 圆拟合 (不变)
    Circle fitCircleRANSAC(const std::vector<std::pair<float, float>>& points,
                           int max_iter, float thresh)
    {
        if (points.size() < 3) return {0,0,0};
        static std::default_random_engine gen(std::random_device{}());
        std::uniform_int_distribution<size_t> dis(0, points.size()-1);
        Circle best_circle{0,0,0};
        int best_inliers = 0;
        for (int iter = 0; iter < max_iter; ++iter) {
            size_t i1 = dis(gen), i2 = dis(gen), i3 = dis(gen);
            if (i1 == i2 || i1 == i3 || i2 == i3) continue;
            Circle c = circleFromThreePoints(points[i1], points[i2], points[i3]);
            if (c.r <= 0) continue;
            int inliers = 0;
            for (const auto& p : points) {
                float d = std::hypot(p.first - c.cx, p.second - c.cy);
                if (std::abs(d - c.r) < thresh) inliers++;
            }
            if (inliers > best_inliers) {
                best_inliers = inliers;
                best_circle = c;
            }
        }
        if (best_inliers < 6) return {0,0,0};
        return best_circle;
    }

    // 圆形置信度 (不变)
    bool isCircleLike(const std::vector<std::pair<float, float>>& points,
                      const Circle& circle,
                      double max_mean_error, double min_angle_span)
    {
        if (points.empty()) return false;
        double sum_err = 0.0;
        for (const auto& p : points) {
            double d = std::hypot(p.first - circle.cx, p.second - circle.cy);
            sum_err += std::abs(d - circle.r);
        }
        double mean_err = sum_err / points.size();
        if (mean_err > max_mean_error) return false;

        std::vector<double> angles;
        for (const auto& p : points) {
            double angle = std::atan2(p.second - circle.cy, p.first - circle.cx);
            angles.push_back(angle);
        }
        std::sort(angles.begin(), angles.end());
        double span = angles.back() - angles.front();
        if (span < min_angle_span) return false;
        return true;
    }

    // 墙角过滤 (不变)
    bool isCornerOrWall(const std::vector<std::pair<float, float>>& points,
                        const Circle& circle,
                        double eigenvalue_ratio_thresh,
                        double min_angle_coverage,
                        double max_radius_stddev)
    {
        if (points.size() < 5) return false;
        double mean_x = 0, mean_y = 0;
        for (const auto& p : points) { mean_x += p.first; mean_y += p.second; }
        mean_x /= points.size(); mean_y /= points.size();
        double cxx = 0, cyy = 0, cxy = 0;
        for (const auto& p : points) {
            double dx = p.first - mean_x, dy = p.second - mean_y;
            cxx += dx*dx; cyy += dy*dy; cxy += dx*dy;
        }
        cxx /= points.size(); cyy /= points.size(); cxy /= points.size();
        double trace = cxx + cyy;
        double det = cxx * cyy - cxy * cxy;
        double eigenvalue1 = (trace + std::sqrt(trace*trace - 4*det)) / 2.0;
        double eigenvalue2 = (trace - std::sqrt(trace*trace - 4*det)) / 2.0;
        if (eigenvalue1 < 1e-6) return false;
        if ((eigenvalue2 / eigenvalue1) < eigenvalue_ratio_thresh) return true;

        std::vector<double> dists;
        for (const auto& p : points)
            dists.push_back(std::hypot(p.first - circle.cx, p.second - circle.cy));
        double sum = 0, sum2 = 0;
        for (double d : dists) { sum += d; sum2 += d*d; }
        double mean = sum / dists.size();
        double stddev = std::sqrt(sum2 / dists.size() - mean*mean);
        if (stddev > max_radius_stddev) return true;

        std::vector<double> angs;
        for (const auto& p : points)
            angs.push_back(std::atan2(p.second - mean_y, p.first - mean_x));
        std::sort(angs.begin(), angs.end());
        double span = angs.back() - angs.front();
        if (span < min_angle_coverage) return true;
        return false;
    }

    // ---------- 根据平均距离自动计算推荐参数 ----------
    struct TuningParams {
        double dbscan_eps;
        int dbscan_min_pts;
        double ransac_thresh;
        double final_min_radius;
        double final_max_radius;
        double max_mean_error;
        double min_angle_span;
        double match_distance;
        double kalman_q;
        double kalman_r;
        int confirm_frames;
        int max_missed;
    };

    TuningParams getAutoParams(double avg_distance)
    {
        double th_1 = this->get_parameter("range_1").as_double();
        double th_2 = this->get_parameter("range_2").as_double();
        double th_3 = this->get_parameter("range_3").as_double();
        double th_4 = this->get_parameter("range_4").as_double();
    //    double th_5 = this->get_parameter("range_5").as_double();
        TuningParams p;
        if (avg_distance < th_1) {  // 近处 (<0.3m)
            p.dbscan_eps = 0.035;       // 更小聚类半径，防止将花盆点云过度合并到其他物体
            p.dbscan_min_pts = 4;       // 降低点数要求
            p.ransac_thresh = 0.008;
            p.final_min_radius = 0.06;
            p.final_max_radius = 0.12;
            p.max_mean_error = 0.008;
            p.min_angle_span = 0.3;     // 放宽角度跨度，允许部分遮挡
            p.match_distance = 0.25;
            p.kalman_q = 0.002;
            p.kalman_r = 0.08;
            p.confirm_frames = 1;       // 快速响应
            p.max_missed = 5;
        } else if (avg_distance < th_2) { // 中距 (<0.4m)
            p.dbscan_eps = 0.05;
            p.dbscan_min_pts = 5;
            p.ransac_thresh = 0.01;
            p.final_min_radius = 0.07;
            p.final_max_radius = 0.10;
            p.max_mean_error = 0.01;
            p.min_angle_span = 0.4;
            p.match_distance = 0.3;
            p.kalman_q = 0.005;
            p.kalman_r = 0.12;
            p.confirm_frames = 2;
            p.max_missed = 10;
        } else if (avg_distance < th_3) { // 中距 (<0.7m)
            p.dbscan_eps = 0.05;
            p.dbscan_min_pts = 5;
            p.ransac_thresh = 0.01;
            p.final_min_radius = 0.07;
            p.final_max_radius = 0.10;
            p.max_mean_error = 0.01;
            p.min_angle_span = 0.4;
            p.match_distance = 0.3;
            p.kalman_q = 0.005;
            p.kalman_r = 0.1;
            p.confirm_frames = 2;
            p.max_missed = 10;
            // 核心：放宽分割、降低左簇点数门槛、收紧杂弧、过滤远噪点
// p.dbscan_eps = 0.07;
// p.dbscan_min_pts = 4;        // 左盆缺点，降低最小簇点数，防止被删掉
// p.ransac_thresh = 0.02;
// p.final_min_radius = 0.06;
// p.final_max_radius = 0.10;
// p.max_mean_error = 0.018;
// p.min_angle_span = 0.4;      // 放宽一点，接纳左侧不完整圆弧
// p.match_distance = 0.25;     // 双盆近距，缩小目标匹配半径，防止互相覆盖
// p.kalman_q = 0.005;
// p.kalman_r = 0.12;           // 稍微加大观测噪声，抗左盆点云波动
// p.confirm_frames = 3;
// p.max_missed = 15;           // 容错拉长，单帧丢左盆不会立刻消失
        } else if (avg_distance < th_4) { // 中距 (<1.0m)
            p.dbscan_eps = 0.05;
            p.dbscan_min_pts = 5;
            p.ransac_thresh = 0.01;
            p.final_min_radius = 0.07;
            p.final_max_radius = 0.10;
            p.max_mean_error = 0.01;
            p.min_angle_span = 0.4;
            p.match_distance = 0.3;
            p.kalman_q = 0.005;
            p.kalman_r = 0.1;
            p.confirm_frames = 2;
            p.max_missed = 10;
        } else {  // 远处 (>1.0m)
            p.dbscan_eps = 0.07;
            p.dbscan_min_pts = 6;
            p.ransac_thresh = 0.015;
            p.final_min_radius = 0.06;
            p.final_max_radius = 0.12;
            p.max_mean_error = 0.015;
            p.min_angle_span = 0.5;
            p.match_distance = 0.35;
            p.kalman_q = 0.01;
            p.kalman_r = 0.15;
            p.confirm_frames = 3;
            p.max_missed = 15;
        }
        return p;
    }

    // ---------- 主回调 ----------
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!msg || msg->ranges.empty()) return;

        // 1. 中值滤波
        int median_window = this->get_parameter("median_window").as_int();
        std::vector<float> ranges_filtered = medianFilter(msg->ranges, median_window);

        // 2. 点云转换
        std::vector<std::pair<float, float>> points = convertToPoints(msg, ranges_filtered);
        if (points.size() < 10) return;

        // 计算整帧点云的平均距离（用于自动调参）
        double avg_distance = 0.0;
        for (const auto& pt : points) {
            avg_distance += std::hypot(pt.first, pt.second);
        }
        avg_distance /= points.size();

        // 获取自动调参开关
        bool auto_tune = this->get_parameter("auto_tune").as_bool();
        TuningParams params;
        if (auto_tune) {
            params = getAutoParams(avg_distance);
        } else {
            // 使用用户设定的参数
            params.dbscan_eps = this->get_parameter("dbscan_eps").as_double();
            params.dbscan_min_pts = this->get_parameter("dbscan_min_pts").as_int();
            params.ransac_thresh = this->get_parameter("ransac_thresh").as_double();
            params.final_min_radius = this->get_parameter("final_min_radius").as_double();
            params.final_max_radius = this->get_parameter("final_max_radius").as_double();
            params.max_mean_error = this->get_parameter("max_mean_error").as_double();
            params.min_angle_span = this->get_parameter("min_angle_span").as_double();
            params.match_distance = this->get_parameter("match_distance").as_double();
            params.kalman_q = this->get_parameter("kalman_q").as_double();
            params.kalman_r = this->get_parameter("kalman_r").as_double();
            params.confirm_frames = this->get_parameter("confirm_frames").as_int();
            params.max_missed = this->get_parameter("max_missed").as_int();
        }

        // 3. DBSCAN 聚类
        auto clusters = dbscanClustering(points, params.dbscan_eps, params.dbscan_min_pts);

        // 4. 圆拟合与基础过滤
        double max_mean_err = params.max_mean_error;
        double min_angle_span = params.min_angle_span;
        bool enable_corner = this->get_parameter("enable_corner_filter").as_bool();
        double wall_ratio = this->get_parameter("wall_eigenvalue_ratio").as_double();
        double min_angle_cov = this->get_parameter("min_angle_coverage").as_double();
        double max_radius_stddev = this->get_parameter("max_radius_stddev").as_double();
        int ransac_iter = this->get_parameter("ransac_iter").as_int();
        double ransac_thresh = params.ransac_thresh;
        double min_range = this->get_parameter("min_range").as_double();
        double max_range = this->get_parameter("max_range").as_double();
        double final_min_r = params.final_min_radius;
        double final_max_r = params.final_max_radius;

        std::vector<Circle> raw_circles;
        for (const auto& cluster : clusters) {
            if (cluster.size() < 5) continue;
            Circle circle = fitCircleRANSAC(cluster, ransac_iter, ransac_thresh);
            if (circle.r <= 0) continue;
            if (circle.r < final_min_r || circle.r > final_max_r) continue;
            if (!isCircleLike(cluster, circle, max_mean_err, min_angle_span)) continue;
            if (enable_corner && isCornerOrWall(cluster, circle, wall_ratio, min_angle_cov, max_radius_stddev)) continue;
            double dist = std::hypot(circle.cx, circle.cy);
            if (dist < min_range || dist > max_range) continue;
            raw_circles.push_back(circle);
        }

        // 5. 跟踪与卡尔曼滤波 (使用自动/手动参数)
        double match_dist = params.match_distance;
        int confirm_frames = params.confirm_frames;
        int max_missed = params.max_missed;
        double kalman_q = params.kalman_q;
        double kalman_r = params.kalman_r;

        for (auto& t : tracked_targets_) t.matched_this_frame = false;
        std::vector<bool> matched_circle(raw_circles.size(), false);
        for (auto& target : tracked_targets_) {
            float best_dist = match_dist;
            int best_idx = -1;
            for (size_t i = 0; i < raw_circles.size(); ++i) {
                if (matched_circle[i]) continue;
                float dx = raw_circles[i].cx - target.kf.getX();
                float dy = raw_circles[i].cy - target.kf.getY();
                float d = std::hypot(dx, dy);
                if (d < best_dist) { best_dist = d; best_idx = i; }
            }
            if (best_idx >= 0) {
                matched_circle[best_idx] = true;
                target.matched_this_frame = true;
                target.missed_count = 0;
                target.confirmed_count++;
                target.kf.update(raw_circles[best_idx].cx, raw_circles[best_idx].cy, raw_circles[best_idx].r);
            } else {
                target.missed_count++;
                target.confirmed_count = 0;
            }
        }
        for (size_t i = 0; i < raw_circles.size(); ++i) {
            if (matched_circle[i]) continue;
            TrackedTarget new_target;
            new_target.id = next_id_++;
            new_target.kf = KalmanFilter(kalman_q, kalman_r);
            new_target.kf.init(raw_circles[i].cx, raw_circles[i].cy, raw_circles[i].r);
            new_target.missed_count = 0;
            new_target.confirmed_count = 1;
            new_target.matched_this_frame = true;
            tracked_targets_.push_back(new_target);
        }
        tracked_targets_.erase(std::remove_if(tracked_targets_.begin(), tracked_targets_.end(),
            [max_missed](const TrackedTarget& t) { return t.missed_count > max_missed; }),
            tracked_targets_.end());

        // 6. 发布
        radar_msgs::msg::RadarTargetArray array_msg;
        int valid_count = 0;
        for (const auto& target : tracked_targets_) {
            if (target.confirmed_count < confirm_frames) continue;
            float fx = target.kf.getX(), fy = target.kf.getY(), fr = target.kf.getR();
            float dist = std::hypot(fx, fy);
            float angle = std::atan2(fy, fx);
            if (fr < final_min_r || fr > final_max_r) continue;
            if (dist < min_range || dist > max_range) continue;
            radar_msgs::msg::RadarTarget target_msg;
            target_msg.n = target.id;
            target_msg.x = fx; target_msg.y = fy; target_msg.radius = fr;
            target_msg.r = dist; target_msg.phi = angle;
            array_msg.array.push_back(target_msg);
            valid_count++;
        }
        target_pub_->publish(array_msg);
        if (valid_count > 0) {
            RCLCPP_INFO(this->get_logger(), "检测到 %d 个花盆 (平均距离=%.2f m, auto_tune=%s)", valid_count, avg_distance, auto_tune?"on":"off");
            for (const auto& t : array_msg.array) {
                RCLCPP_INFO(this->get_logger(),
                    "花盆 ID=%d: 中心=(%.2f, %.2f), 半径=%.2fm, 距离=%.2fm, 角度=%.1f°",
                    t.n, t.x, t.y, t.radius, t.r, t.phi * 180.0 / M_PI);
            }
        }
    }

    std::vector<TrackedTarget> tracked_targets_;
    int next_id_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<radar_msgs::msg::RadarTargetArray>::SharedPtr target_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}