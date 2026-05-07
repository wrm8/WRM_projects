#include <rclcpp/rclcpp.hpp>//所有ROS节点的必需品，提供节点初始化、发布订阅、参数获取等基础功能。
//#include <image_transport/image_transport.hpp>//专门用于图像传输，支持压缩传输，减少网络带宽占用。
//#include <cv_bridge/cv_bridge.hpp>//在OpenCV的cv::Mat和ROS的sensor_msgs/Image之间转换。
/*******定义ROS中常用的传感器消息类型。**********/
#include <sensor_msgs/msg/image.hpp>// 原始图像消息
#include <sensor_msgs/msg/compressed_image.hpp>// 压缩图像消息
#include <sensor_msgs/image_encodings.hpp>// 图像编码格式
/*******OpenCV 头文件（图像处理、摄像头操作）**********/
#include <opencv2/opencv.hpp>      // OpenCV 核心库
#include <opencv2/videoio.hpp>     // 视频输入输出

#include <vector>//用于使用动态数组（vector）

class CompressedCameraNode : public rclcpp::Node
{
public:
    CompressedCameraNode() : Node("camera_node")// 调用父类构造函数
    {
        // 一行搞定声明和赋值
        device_id_ = this->declare_parameter("device_id", 0);
        frame_width_ = this->declare_parameter("frame_width", 640);
        frame_height_ = this->declare_parameter("frame_height", 480);
        fps_ = this->declare_parameter("fps", 30);
        jpeg_quality_ = this->declare_parameter("jpeg_quality", 95);
        // // 步骤1：声明参数（告诉ROS2我有这个参数）
        // this->declare_parameter("device_id", 0);
        // this->declare_parameter("frame_width", 640);
        // this->declare_parameter("frame_height", 480);
        // this->declare_parameter("fps", 30);
        // this->declare_parameter("jpeg_quality", 95);
        // // 步骤2：获取参数值（从ROS2参数服务器读取）
        // device_id_ = this->get_parameter("device_id").as_int();
        // frame_width_ = this->get_parameter("frame_width").as_int();
        // frame_height_ = this->get_parameter("frame_height").as_int();
        // fps_ = this->get_parameter("fps").as_int();
        // jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        
        // // 新增video_device参数声明
        // video_device_ = this->declare_parameter("video_device", "");

        // 创建压缩图像发布器
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "image_raw/compressed", 10);//话题名称"image_raw/compressed"
        //如果订阅者处理不过来，最多缓存10条消息，超过10条，最早的消息会被丢弃

        RCLCPP_INFO(this->get_logger(), "Starting USB camera with device ID: %d", device_id_);
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d, FPS: %d, JPEG Quality: %d", 
                    frame_width_, frame_height_, fps_, jpeg_quality_);
    }
    
    bool initializeCamera()
    {
        // // 优先使用video_device参数
        // if (!video_device_.empty()) {
        //     cap_.open(video_device_, cv::CAP_V4L2);
        // } else {
        //     cap_.open(device_id_, cv::CAP_V4L2);
        // }

        // if (!cap_.isOpened())
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open USB camera");
        //     return false;
        // }
        // 打开摄像头
        cap_.open(device_id_, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open USB camera with device ID: %d", device_id_);
            return false;
        }
        
        // 设置相机参数
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
        cap_.set(cv::CAP_PROP_FPS, fps_);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        
        return true;
    }
    
    void run()
    {
        cv::Mat frame;
        auto rate = std::make_shared<rclcpp::Rate>(fps_);//频率控制器
        
        // JPEG压缩参数
        std::vector<int> compression_params;
        compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params.push_back(jpeg_quality_);
        
        while (rclcpp::ok())
        {
            if (!cap_.read(frame))
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to read frame from camera");
                break;
            }
            
            if (frame.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Empty frame received");
                continue;
            }
            
            // 创建压缩图像消息
            auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
            compressed_msg->header.stamp = this->now();
            compressed_msg->header.frame_id = "camera_frame";
            compressed_msg->format = "jpeg";
            
            // 压缩图像
            if (cv::imencode(".jpg", frame, compressed_msg->data, compression_params))
            {
                compressed_pub_->publish(*compressed_msg);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to compress image");
            }
            
            rate->sleep();
        }
        
        cap_.release();
    }
    
private:
    cv::VideoCapture cap_;//OpenCV摄像头对象	从USB摄像头读取图像
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;//发布者智能指针	向ROS话题发布压缩图像
    
    int device_id_;//摄像头设备ID（0=/dev/video0）
    int frame_width_;
    int frame_height_;
    int fps_;//帧率
    int jpeg_quality_;//JPEG压缩质量（0-100）
   // std::string video_device_; // 新增成员变量
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);//初始化ROS
    auto node = std::make_shared<CompressedCameraNode>();
    
    if (node->initializeCamera())
    {
        node->run();
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize camera");
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}