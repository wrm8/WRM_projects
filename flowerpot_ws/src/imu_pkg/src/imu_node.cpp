#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include "imu_pkg/serial_port.h"
//#include <serial/serial.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <string>

// 编译选项：取消注释则只输出 yaw 角度
 #define ONLY_YAW 1

enum class ReadState {
    WAIT_FOR_55,    // 等待 0x55
    READ_44_BYTES   // 读取 44 字节
};

class ImuNode : public rclcpp::Node {
public:
    ImuNode() : Node("imu_node"), current_state_(ReadState::WAIT_FOR_55), is_serial_connected_(false)
    {
        // 直接声明并赋值1
        port_ = this->declare_parameter<std::string>("port", "/dev/imu");
        baudrate_ = this->declare_parameter<int>("baudrate", 9600);
        // // 声明参数
        // this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        // this->declare_parameter<int>("baudrate", 9600);
        
        // port_ = this->get_parameter("port").as_string();
        // baudrate_ = this->get_parameter("baudrate").as_int();
        
        // 创建发布者
#ifdef ONLY_YAW
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float32>("yaw_angle", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing yaw angle to /yaw_angle");
#else
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing IMU data to /imu/data");

        // 设置 IMU 消息的坐标系
        imu_msg_.header.frame_id = "imu_link";
        
        // 设置协方差（默认值）
        imu_msg_.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
        imu_msg_.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
        imu_msg_.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
#endif
        
        // 初始化串口
        is_serial_connected_ = initSerial();
    }
    
    ~ImuNode() {
        if (serial_.isOpen()) {
            serial_.close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }
    
    void run() {
        if (!is_serial_connected_) {
            RCLCPP_ERROR(this->get_logger(), "Serial port not connected, cannot run");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "IMU node started, reading data...");
        
        rclcpp::Rate loop_rate(50);  // 50Hz 读取频率
        
        while (rclcpp::ok() && is_serial_connected_) {
            try {
                readData();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Read failed: %s", e.what());
            }
            loop_rate.sleep();
        }
    }

private:
    bool initSerial() {
        try {//try-catch 异常处理
            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);
            serial_.open();// 可能出错的代码
            
            if (serial_.isOpen()) {
                RCLCPP_INFO(this->get_logger(), "Serial port %s opened, baudrate=%d", 
                            port_.c_str(), baudrate_);
                return true;
            }
            // } catch (const serial::IOException& e)// 专门处理串口IO异常
            // {
            //      RCLCPP_ERROR(this->get_logger(), "Cannot open serial port %s: %s", 
            //                 port_.c_str(), e.what());
            } catch (const std::exception& e) // 匹配基类异常
            {
               RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
            }
        return false;
    }
    
    void processData(const std::vector<uint8_t>& frame) {
        if (frame.size() < 11) return;
        
        // 校验和计算（根据手册，校验和为前面10个字节的和）//Lambda 表达式（匿名函数）
        //[&] 会捕获所有在 Lambda 外部定义的、且在 Lambda 内部使用的变量。也就是捕获才能进来使用
        //[=] 按值捕获所有，可以访问，但捕获的是拷贝
        auto calc_checksum = [&](size_t start) -> uint8_t {
            uint8_t sum = 0;
            for (size_t i = 0; i < 10; i++) {
                sum += frame[start + i];
            }
            return sum;
        };
        
#ifdef ONLY_YAW
        // 只输出 yaw 角度
        // 角度数据格式：0x55, 0x53, RollL, RollH, PitchL, PitchH, YawL, YawH, VL, VH, SUM
        if (frame[1] == 0x53) {
            uint8_t checksum = calc_checksum(0);//校验和
            if (checksum == frame[10]) {
                int16_t yaw_raw = (frame[7] << 8) | frame[6];
                float yaw = static_cast<float>(yaw_raw) / 32768.0f * 180.0f;//显示转换
                
                RCLCPP_INFO(this->get_logger(), "Yaw: %.2f", yaw);
                
                std_msgs::msg::Float32 yaw_msg;
                yaw_msg.data = yaw;
                yaw_pub_->publish(yaw_msg);
            } else {
                RCLCPP_WARN(this->get_logger(), "Checksum error for yaw frame");
            }
        }
#else
        // 完整 IMU 数据
        imu_msg_.header.stamp = this->now();
        
        // 1. 角度数据（Roll, Pitch, Yaw）- 类型 0x53
        if (frame[1] == 0x53) {
            uint8_t checksum = calc_checksum(0);
            if (checksum == frame[10]) {
                int16_t roll_raw = (frame[3] << 8) | frame[2];
                int16_t pitch_raw = (frame[5] << 8) | frame[4];
                int16_t yaw_raw = (frame[7] << 8) | frame[6];
                
                float roll = static_cast<float>(roll_raw) / 32768.0f * 180.0f;
                float pitch = static_cast<float>(pitch_raw) / 32768.0f * 180.0f;
                float yaw = static_cast<float>(yaw_raw) / 32768.0f * 180.0f;
                
                // 转换为四元数
                tf2::Quaternion q;
                q.setRPY(roll * M_PI / 180.0, 
                         pitch * M_PI / 180.0, 
                         yaw * M_PI / 180.0);
                q.normalize();
                
                imu_msg_.orientation.x = q.x();
                imu_msg_.orientation.y = q.y();
                imu_msg_.orientation.z = q.z();
                imu_msg_.orientation.w = q.w();
                
                RCLCPP_DEBUG(this->get_logger(), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", 
                             roll, pitch, yaw);
            }
        }
        
        // 2. 角速度数据（陀螺仪）- 类型 0x52
        if (frame[12] == 0x52) {
            uint8_t checksum = calc_checksum(0);
            if (checksum == frame[10]) {
                int16_t wx_raw = (frame[3] << 8) | frame[2];
                int16_t wy_raw = (frame[5] << 8) | frame[4];
                int16_t wz_raw = (frame[7] << 8) | frame[6];
                
                // 量程 ±2000°/s，转换为 rad/s
                imu_msg_.angular_velocity.x = static_cast<float>(wx_raw) / 32768.0f * 2000.0f * M_PI / 180.0;
                imu_msg_.angular_velocity.y = static_cast<float>(wy_raw) / 32768.0f * 2000.0f * M_PI / 180.0;
                imu_msg_.angular_velocity.z = static_cast<float>(wz_raw) / 32768.0f * 2000.0f * M_PI / 180.0;
            }
        }
        
        // 3. 加速度数据 - 类型 0x51
        if (frame[23] == 0x51) {
            uint8_t checksum = calc_checksum(0);
            if (checksum == frame[10]) {
                int16_t ax_raw = (frame[3] << 8) | frame[2];
                int16_t ay_raw = (frame[5] << 8) | frame[4];
                int16_t az_raw = (frame[7] << 8) | frame[6];
                
                // 量程 ±16g，转换为 m/s²
                imu_msg_.linear_acceleration.x = static_cast<float>(ax_raw) / 32768.0f * 16.0f * 9.81;
                imu_msg_.linear_acceleration.y = static_cast<float>(ay_raw) / 32768.0f * 16.0f * 9.81;
                imu_msg_.linear_acceleration.z = static_cast<float>(az_raw) / 32768.0f * 16.0f * 9.81;
            }
        }
        
        // 发布 IMU 消息
        imu_pub_->publish(imu_msg_);
#endif
    }
    
    void readData() {
        std::vector<uint8_t> buffer(11);
        
        while (serial_.available() >= 11 && rclcpp::ok()) {
            if (current_state_ == ReadState::WAIT_FOR_55) {
                uint8_t byte;
                size_t bytes_read = serial_.read(&byte, 1);
                if (bytes_read == 1 && byte == 0x55) {
                    // 读取第二个字节，确认数据包类型
                     bytes_read = serial_.read(&byte, 1);
                    if (bytes_read == 1 && (byte == 0x53)) {
                    //   // 跳过剩余的前导字节
                    //     uint8_t discard[9];
                    //     serial_.read(discard, sizeof(discard));//读一下给消耗掉
                        current_state_ = ReadState::READ_44_BYTES;
                    }
                }
            } else {
                // 读剩余的 9 字节（加上已读的 2 字节，共 11 字节）
                    size_t bytes_read = serial_.read(buffer.data() + 2, 9);
                    if (bytes_read == 9) {
                    // 重新组装完整帧：buffer[0-1] 是已读的 0x55 0x53
                    // buffer[2-10] 是刚读的 9 字节
                    buffer[0] = 0x55;
                    buffer[1] = 0x53;
                    processData(buffer);
                    }else {
                            RCLCPP_WARN(this->get_logger(), "Expected 9 more bytes, got %zu", bytes_read);
                        }
                        current_state_ = ReadState::WAIT_FOR_55;
                }
                // //先读一字节，确保是帧头
                //     uint8_t first_byte;
                //     size_t n = serial_.read(&first_byte, 1);
                //     if (n == 1 && first_byte == 0x55) {
                //         // 再读 10 字节
                //         size_t bytes_read = serial_.read(buffer.data() + 1, 9);
                //         if (bytes_read == 10) {
                //             buffer[0] = 0x55;
                //             processData(buffer);
                //         } else {
                //             RCLCPP_WARN(this->get_logger(), "Expected 10 more bytes, got %zu", bytes_read);
                //         }
                //     } else {
                //         // 没找到帧头，丢弃并重新同步
                //         RCLCPP_WARN(this->get_logger(), "Sync lost, re-syncing...");
                //     }
                // }
                // current_state_ = ReadState::WAIT_FOR_55;
        }
    }
    
    
    // 串口相关
    SerialPort serial_;
    std::string port_;
    int baudrate_;
    bool is_serial_connected_;
    ReadState current_state_;
    
    // 发布者
#ifdef ONLY_YAW
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr yaw_pub_;
#else
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    sensor_msgs::msg::Imu imu_msg_;
#endif
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);// 初始化 ROS2 客户端库
    auto node = std::make_shared<ImuNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
// main()
//   │
//   ├─ 1. rclcpp::init(argc, argv);            // 初始化 ROS2 客户端库
//   ├─ 2. auto node = std::make_shared<ImuNode>(); // 创建节点对象（智能指针）
//   │       │
//   │       └─ ImuNode 构造函数执行：
//   │           ├─ 声明/获取参数（port, baudrate）
//   │           ├─ 创建发布者（yaw_pub_ 或 imu_pub_）
//   │           ├─ 设置消息默认值（frame_id, 协方差）
//   │           └─ 调用 initSerial() 打开串口 → is_serial_connected_ = true/false
//   │
//   ├─ 3. node->run();                         // 进入主循环
//   │       │
//   │       ├─ 检查 is_serial_connected_，若 false 则返回
//   │       ├─ rclcpp::Rate loop_rate(50);    // 控制循环频率 50Hz
//   │       └─ while (rclcpp::ok() && is_serial_connected_) {
//   │               readData();               // 核心读取与解析
//   │               loop_rate.sleep();
//   │           }
//   │
//   └─ 4. rclcpp::shutdown();                  // 关闭 ROS2
//我描述一下数据流，在main函数中创建智能指针auto node = std::make_shared<ImuNode>();，
//构造函数中自动创建话题，并通过initSerial()初始化串口并异常处理没问题就打开串口，
//调用node->run();首先串口发送的是数据流，数据流会暂存在串口缓冲区，设置读取频率，
//通过readData()进行数据读取和处理;里的传感器第三方库serial_.read(&byte, 1)函数，来进行数据的校验和读取，
//并且边读取，地址中的数据边更新，读取的速度要大于串口接收速度，并且读取过的数据被删除，让新来的数据有地方进来，
//用processData（）来进行数据处理，将处理过的数据存在buffer暂存区。