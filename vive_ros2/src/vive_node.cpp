#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <fstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "json.hpp" // Include nlohmann/json
#include "VRUtils.hpp"
#include "EigenTransforms.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "vive_ros2/msg/vr_controller_data.hpp"

using json = nlohmann::json;

// Constants for configuration
namespace VRConfig {
    constexpr int DEFAULT_QUEUE_SIZE = 150;
    constexpr int BUFFER_SIZE = 1024;
    constexpr int PERFORMANCE_LOG_INTERVAL = 100;
    constexpr int RECONNECTION_DELAY_SECONDS = 1;
    constexpr double POSE_SMOOTHING_FACTOR = 0.9999;
    constexpr int RIGHT_CONTROLLER_ROLE = 0;
    constexpr int LEFT_CONTROLLER_ROLE = 1;
    
    // Coordinate transformation constants (VR Y-up to ROS Z-up)
    constexpr double COORD_TRANSFORM_NEG = -1.0;
}

// Controller role enumeration for better type safety
enum class ControllerRole {
    Right = VRConfig::RIGHT_CONTROLLER_ROLE,
    Left = VRConfig::LEFT_CONTROLLER_ROLE
};

// Socket configuration parameter object
struct SocketConfig {
    std::string address;
    int port;
    
    SocketConfig(const std::string& addr, int p) : address(addr), port(p) {}
};

// VR Controller ROS2 Client - bridges VR input server to ROS2 transforms
class VRControllerClient : public rclcpp::Node {
private:
    int sock;
    struct sockaddr_in serv_addr;
    SocketConfig socketConfig;
    VRControllerData jsonData; // Use the struct for JSON data
    VRControllerData initialPoseLeft;
    VRControllerData initialPoseRight;
    bool triggerButtonPressedLeft = false;
    bool triggerButtonPressedRight = false;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr rel_transform_publisher_;
    rclcpp::Publisher<vive_ros2::msg::VRControllerData>::SharedPtr controller_data_publisher_;
    
    // Role-specific publishers
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr left_abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr left_rel_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr right_abs_transform_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr right_rel_transform_publisher_;

    // Debug logging
    std::ofstream log_file_;
    bool enable_debug_logging_ = true;
    std::string current_test_direction_ = "NONE";
    int direction_index_ = 0;
    std::vector<std::string> test_directions_ = {"NONE", "FORWARD", "BACKWARD", "LEFT", "RIGHT"};
    bool grip_button_was_pressed_left_ = false;
    bool grip_button_was_pressed_right_ = false;

    // 平移测试记录
    struct DirectionTestData {
        bool is_recording = false;
        VRControllerData start_pose;
        VRControllerData current_pose;
        VRControllerData max_displacement;
        int sample_count = 0;
        std::string direction;

        void reset() {
            is_recording = false;
            sample_count = 0;
            max_displacement = VRControllerData();
        }
    };
    DirectionTestData direction_test_;

    // Open debug log file
    void openLogFile() {
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        std::tm* tm_now = std::localtime(&time_t_now);
        
        char filename[128];
        snprintf(filename, sizeof(filename), "/tmp/controller_debug_%04d%02d%02d_%02d%02d%02d.log",
                 tm_now->tm_year + 1900, tm_now->tm_mon + 1, tm_now->tm_mday,
                 tm_now->tm_hour, tm_now->tm_min, tm_now->tm_sec);
        
        log_file_.open(filename, std::ios::out | std::ios::app);
        if (log_file_.is_open()) {
            RCLCPP_INFO(this->get_logger(), "调试日志文件已打开: %s", filename);
            log_file_ << "========================================\n";
            log_file_ << "VR Controller Debug Log\n";
            log_file_ << "========================================\n\n";
        } else {
            RCLCPP_ERROR(this->get_logger(), "无法打开调试日志文件: %s", filename);
        }
    }

    // Log controller data to file
    void logControllerDataToFile(const VRControllerData& absData, const VRControllerData& relData) {
        if (!log_file_.is_open() || !enable_debug_logging_) return;
        
        ControllerRole role = static_cast<ControllerRole>(absData.role);
        std::string controller_name = (role == ControllerRole::Left) ? "LEFT" : "RIGHT";
        
        log_file_ << "----------------------------------------\n";
        log_file_ << "时间: " << absData.time << "\n";
        log_file_ << "手柄: " << controller_name << "\n";
        log_file_ << "测试方向: " << current_test_direction_ << "\n\n";
        
        // Absolute pose (VR原始坐标系)
        log_file_ << "绝对位置 (VR坐标系):\n";
        log_file_ << "  Position: x=" << absData.pose_x 
                  << ", y=" << absData.pose_y 
                  << ", z=" << absData.pose_z << "\n";
        log_file_ << "  Rotation: qx=" << absData.pose_qx 
                  << ", qy=" << absData.pose_qy 
                  << ", qz=" << absData.pose_qz 
                  << ", qw=" << absData.pose_qw << "\n\n";
        
        // Relative pose (VR原始坐标系)
        log_file_ << "相对位置 (VR坐标系):\n";
        log_file_ << "  Position: x=" << relData.pose_x 
                  << ", y=" << relData.pose_y 
                  << ", z=" << relData.pose_z << "\n";
        log_file_ << "  Rotation: qx=" << relData.pose_qx 
                  << ", qy=" << relData.pose_qy 
                  << ", qz=" << relData.pose_qz 
                  << ", qw=" << relData.pose_qw << "\n\n";
        
        // ROS coordinate system (transformed)
        log_file_ << "绝对位置 (ROS坐标系 - 转换后):\n";
        log_file_ << "  Position: x=" << (VRConfig::COORD_TRANSFORM_NEG * absData.pose_z)
                  << ", y=" << (VRConfig::COORD_TRANSFORM_NEG * absData.pose_x)
                  << ", z=" << absData.pose_y << "\n";
        log_file_ << "  Rotation: qx=" << (VRConfig::COORD_TRANSFORM_NEG * absData.pose_qz)
                  << ", qy=" << (VRConfig::COORD_TRANSFORM_NEG * absData.pose_qx)
                  << ", qz=" << absData.pose_qy
                  << ", qw=" << absData.pose_qw << "\n\n";
        
        // Button states
        log_file_ << "按钮状态:\n";
        log_file_ << "  Trigger: " << absData.trigger_button 
                  << " (value: " << absData.trigger << ")\n";
        log_file_ << "  Grip: " << absData.grip_button << "\n";
        log_file_ << "  Menu: " << absData.menu_button << "\n";
        log_file_ << "  Trackpad: button=" << absData.trackpad_button 
                  << ", touch=" << absData.trackpad_touch
                  << ", x=" << absData.trackpad_x 
                  << ", y=" << absData.trackpad_y << "\n";
        
        log_file_ << "\n" << std::flush;
    }

    // 开始方向测试记录
    void startDirectionTest(const VRControllerData& pose) {
        direction_test_.is_recording = true;
        direction_test_.start_pose = pose;
        direction_test_.current_pose = pose;
        direction_test_.direction = current_test_direction_;
        direction_test_.sample_count = 0;
        direction_test_.max_displacement = VRControllerData();

        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "▶▶▶ 开始记录 [%s] 方向测试 ◀◀◀", current_test_direction_.c_str());
        RCLCPP_INFO(this->get_logger(), "起始位置 (VR): x=%.4f, y=%.4f, z=%.4f",
                    pose.pose_x, pose.pose_y, pose.pose_z);
        RCLCPP_INFO(this->get_logger(), "起始位置 (ROS): x=%.4f, y=%.4f, z=%.4f",
                    VRConfig::COORD_TRANSFORM_NEG * pose.pose_z,
                    VRConfig::COORD_TRANSFORM_NEG * pose.pose_x,
                    pose.pose_y);

        if (log_file_.is_open()) {
            log_file_ << "\n================== 方向测试开始 ==================\n";
            log_file_ << "测试方向: " << current_test_direction_ << "\n";
            log_file_ << "起始位置 (VR坐标系): x=" << pose.pose_x
                      << ", y=" << pose.pose_y << ", z=" << pose.pose_z << "\n";
            log_file_ << "起始位置 (ROS坐标系): x=" << (VRConfig::COORD_TRANSFORM_NEG * pose.pose_z)
                      << ", y=" << (VRConfig::COORD_TRANSFORM_NEG * pose.pose_x)
                      << ", z=" << pose.pose_y << "\n";
            log_file_ << "---------------------------------------------------\n";
            log_file_.flush();
        }
    }

    // 更新方向测试数据
    void updateDirectionTest(const VRControllerData& pose) {
        if (!direction_test_.is_recording) return;

        direction_test_.current_pose = pose;
        direction_test_.sample_count++;

        // 计算相对于起始位置的位移 (VR坐标系)
        double dx_vr = pose.pose_x - direction_test_.start_pose.pose_x;
        double dy_vr = pose.pose_y - direction_test_.start_pose.pose_y;
        double dz_vr = pose.pose_z - direction_test_.start_pose.pose_z;

        // 转换到ROS坐标系
        double dx_ros = VRConfig::COORD_TRANSFORM_NEG * dz_vr;  // ROS X = -VR Z
        double dy_ros = VRConfig::COORD_TRANSFORM_NEG * dx_vr;  // ROS Y = -VR X
        double dz_ros = dy_vr;                                   // ROS Z = VR Y

        // 每10个样本输出一次实时数据
        if (direction_test_.sample_count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] 位移(ROS): dx=%.4f, dy=%.4f, dz=%.4f | 样本数: %d",
                        current_test_direction_.c_str(), dx_ros, dy_ros, dz_ros,
                        direction_test_.sample_count);
        }

        // 记录到日志文件
        if (log_file_.is_open() && direction_test_.sample_count % 5 == 0) {
            log_file_ << "样本 #" << direction_test_.sample_count
                      << " | 位移(ROS): dx=" << dx_ros << ", dy=" << dy_ros << ", dz=" << dz_ros << "\n";
        }
    }

    // 结束方向测试并输出摘要
    void endDirectionTest() {
        if (!direction_test_.is_recording) return;

        const auto& start = direction_test_.start_pose;
        const auto& end = direction_test_.current_pose;

        // VR坐标系位移
        double dx_vr = end.pose_x - start.pose_x;
        double dy_vr = end.pose_y - start.pose_y;
        double dz_vr = end.pose_z - start.pose_z;

        // ROS坐标系位移
        double dx_ros = VRConfig::COORD_TRANSFORM_NEG * dz_vr;
        double dy_ros = VRConfig::COORD_TRANSFORM_NEG * dx_vr;
        double dz_ros = dy_vr;

        double total_displacement = std::sqrt(dx_ros*dx_ros + dy_ros*dy_ros + dz_ros*dz_ros);

        // 输出测试摘要到控制台
        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║          [%s] 方向测试完成                              ║",
                    direction_test_.direction.c_str());
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  样本数量: %-44d ║", direction_test_.sample_count);
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  VR坐标系位移:                                           ║");
        RCLCPP_INFO(this->get_logger(), "║    dx = %+.4f                                           ║", dx_vr);
        RCLCPP_INFO(this->get_logger(), "║    dy = %+.4f                                           ║", dy_vr);
        RCLCPP_INFO(this->get_logger(), "║    dz = %+.4f                                           ║", dz_vr);
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  ROS坐标系位移 (机器人坐标系):                           ║");
        RCLCPP_INFO(this->get_logger(), "║    dx = %+.4f (前后方向, +为前)                         ║", dx_ros);
        RCLCPP_INFO(this->get_logger(), "║    dy = %+.4f (左右方向, +为左)                         ║", dy_ros);
        RCLCPP_INFO(this->get_logger(), "║    dz = %+.4f (上下方向, +为上)                         ║", dz_ros);
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  总位移距离: %.4f 米                                    ║", total_displacement);
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "\n");

        // 输出到日志文件
        if (log_file_.is_open()) {
            log_file_ << "\n================== 方向测试摘要 ==================\n";
            log_file_ << "测试方向: " << direction_test_.direction << "\n";
            log_file_ << "样本数量: " << direction_test_.sample_count << "\n\n";

            log_file_ << "起始位置 (VR): x=" << start.pose_x << ", y=" << start.pose_y << ", z=" << start.pose_z << "\n";
            log_file_ << "结束位置 (VR): x=" << end.pose_x << ", y=" << end.pose_y << ", z=" << end.pose_z << "\n\n";

            log_file_ << "VR坐标系位移:\n";
            log_file_ << "  dx = " << dx_vr << "\n";
            log_file_ << "  dy = " << dy_vr << "\n";
            log_file_ << "  dz = " << dz_vr << "\n\n";

            log_file_ << "ROS坐标系位移 (机器人坐标系):\n";
            log_file_ << "  dx = " << dx_ros << " (前后方向, +为前)\n";
            log_file_ << "  dy = " << dy_ros << " (左右方向, +为左)\n";
            log_file_ << "  dz = " << dz_ros << " (上下方向, +为上)\n\n";

            log_file_ << "总位移距离: " << total_displacement << " 米\n";
            log_file_ << "=================================================\n\n";
            log_file_.flush();
        }

        direction_test_.reset();
    }

    // Initialize socket address structure for TCP connection
    void setupSocket() {
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(socketConfig.port);
        if(inet_pton(AF_INET, socketConfig.address.c_str(), &serv_addr.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid address/ Address not supported");
            exit(EXIT_FAILURE);
        }
    }

    void createPublishers() {
        // Create publishers with role-specific topic names
        abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        controller_data_publisher_ = this->create_publisher<vive_ros2::msg::VRControllerData>("controller_data", 10);
        
        // Create role-specific publishers for better topic organization
        left_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        left_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("left_vr/vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        right_abs_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_abs", VRConfig::DEFAULT_QUEUE_SIZE);
        right_rel_transform_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("right_vr/vive_pose_rel", VRConfig::DEFAULT_QUEUE_SIZE);
        
        RCLCPP_INFO(this->get_logger(), "Publishers created with role-based topic names.");
    }

    // Create TCP socket for VR server communication
    void establishServerConnection() {
        sock = socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            RCLCPP_ERROR(this->get_logger(), "Socket creation error");
            exit(EXIT_FAILURE);
        }

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection Failed");
            close(sock);
            sock = -1;
        }
    }

    void reconnect() {
        close(sock);
        sock = -1;
        while (sock < 0) {
            // Exponential backoff could be implemented here for production use
            std::this_thread::sleep_for(std::chrono::seconds(VRConfig::RECONNECTION_DELAY_SECONDS));
            establishServerConnection();
        }
        RCLCPP_INFO(this->get_logger(), "Reconnected to server.");
    }

    // Transform from VR coordinate system (Y-up) to ROS (Z-up)
    void transformVRToROS(const VRControllerData& vrData, geometry_msgs::msg::TransformStamped& transform) {
        transform.transform.translation.x = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_z;
        transform.transform.translation.y = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_x;
        transform.transform.translation.z = vrData.pose_y;
        transform.transform.rotation.x = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_qz;
        transform.transform.rotation.y = VRConfig::COORD_TRANSFORM_NEG * vrData.pose_qx;
        transform.transform.rotation.z = vrData.pose_qy;
        transform.transform.rotation.w = vrData.pose_qw;
    }

    std::string generateFrameId(ControllerRole role, bool isRelative) {
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        std::string suffix = isRelative ? "vive_pose_rel" : "vive_pose_abs";
        return prefix + "/" + suffix;
    }

    void publishTransform(const VRControllerData& pose, bool isRelative = false) {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        
        ControllerRole role = static_cast<ControllerRole>(pose.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        std::string topic_name = (isRelative) ? "vive_pose_rel" : "vive_pose_abs";
        
        // Set child_frame_id with the appropriate prefix
        transformStamped.child_frame_id = generateFrameId(role, isRelative);

        // Transform from VR coordinate system (Y-up) to ROS (Z-up)
        transformVRToROS(pose, transformStamped);

        // Publish to TF with role-based frame naming
        tf_broadcaster_->sendTransform(transformStamped);
        
        // Create a new message to publish to the role-specific topic
        auto roleSpecificMsg = transformStamped;
        
        // Publish to the appropriate role-specific topic
        RCLCPP_INFO(this->get_logger(), "Publishing transform for role: %d", pose.role);
        if (role == ControllerRole::Left) {
            if (isRelative) {
                left_rel_transform_publisher_->publish(roleSpecificMsg);
            } else {
                left_abs_transform_publisher_->publish(roleSpecificMsg);
            }
        } else { // Right controller
            if (isRelative) {
                right_rel_transform_publisher_->publish(roleSpecificMsg);
            } else {
                right_abs_transform_publisher_->publish(roleSpecificMsg);
            }
        }
        
        // Also publish to generic topics for backward compatibility
        if (isRelative) {
            rel_transform_publisher_->publish(roleSpecificMsg);
        } else {
            abs_transform_publisher_->publish(roleSpecificMsg);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Published transform to %s with child_frame_id %s", 
                  (isRelative ? "vive_pose_rel" : "vive_pose_abs"), 
                  transformStamped.child_frame_id.c_str());
    }

    VRControllerData calculateRelativePose(const VRControllerData& initial, const VRControllerData& current) {
        // Use the new Eigen-based utility function
        return VRUtils::calculateRelativePose(initial, current);
    }

    VRControllerData filterPose(const VRControllerData& pose) {
        // Low-pass filter with controller-specific state tracking
        static VRControllerData prevPoseLeft;
        static VRControllerData prevPoseRight;
        static bool initialized[2] = {false, false};
        
        // Get appropriate previous pose based on controller role
        VRControllerData& prevPose = (pose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? prevPoseLeft : prevPoseRight;
        
        // Initialize if this is the first data for this controller
        if (!initialized[pose.role]) {
            prevPose = pose;
            initialized[pose.role] = true;
            return pose; // Return unfiltered for first sample
        }

        // Use the new Eigen-based filtering utility
        VRControllerData filteredPose = VRUtils::filterPose(pose, prevPose, VRConfig::POSE_SMOOTHING_FACTOR);
        
        // Store this pose for next iteration
        prevPose = filteredPose;
        
        return filteredPose;
    }



public:
    VRControllerClient(std::string addr, int p) : Node("client_node"), sock(-1), socketConfig(addr, p) {
        setupSocket();
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        createPublishers();
        openLogFile();
    }

    ~VRControllerClient() {
        if (sock != -1) {
            close(sock);
        }
        if (log_file_.is_open()) {
            log_file_ << "\n========================================\n";
            log_file_ << "Log session ended\n";
            log_file_ << "========================================\n";
            log_file_.close();
            RCLCPP_INFO(this->get_logger(), "调试日志文件已关闭");
        }
    }

    void assignCoordinates(const VRControllerData& data, geometry_msgs::msg::TransformStamped& transform) {
        transform.transform.translation.x = data.pose_x;
        transform.transform.translation.y = data.pose_y;
        transform.transform.translation.z = data.pose_z;
        transform.transform.rotation.x = data.pose_qx;
        transform.transform.rotation.y = data.pose_qy;
        transform.transform.rotation.z = data.pose_qz;
        transform.transform.rotation.w = data.pose_qw;
    }

    void populateControllerMessage(const VRControllerData& absoluteData, const VRControllerData& relativeData, 
                                 vive_ros2::msg::VRControllerData& msg) {
        msg.grip_button = absoluteData.grip_button;
        msg.trigger_button = absoluteData.trigger_button;
        msg.trackpad_button = absoluteData.trackpad_button;
        msg.trackpad_touch = absoluteData.trackpad_touch;
        msg.menu_button = absoluteData.menu_button;
        msg.trackpad_x = absoluteData.trackpad_x;
        msg.trackpad_y = absoluteData.trackpad_y;
        msg.trigger = absoluteData.trigger;
        msg.role = absoluteData.role;
        msg.time = absoluteData.time;

        // Determine prefix based on controller role
        ControllerRole role = static_cast<ControllerRole>(absoluteData.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";

        // Absolute pose data
        msg.abs_pose.header.stamp = this->get_clock()->now();
        msg.abs_pose.header.frame_id = "world";
        msg.abs_pose.child_frame_id = prefix + "/vive_pose_abs";
        assignCoordinates(absoluteData, msg.abs_pose);

        // Relative pose data
        msg.rel_pose.header.stamp = this->get_clock()->now();
        msg.rel_pose.header.frame_id = "world";
        msg.rel_pose.child_frame_id = prefix + "/vive_pose_rel";
        assignCoordinates(relativeData, msg.rel_pose);
    }

    void publishControllerData(const VRControllerData& absoluteData, const VRControllerData& relativeData) {
        vive_ros2::msg::VRControllerData msg;
        populateControllerMessage(absoluteData, relativeData, msg);

        // Publish to the controller data topic with role-specific frame_ids
        controller_data_publisher_->publish(msg);
        
        // Log to file with detailed information
        logControllerDataToFile(absoluteData, relativeData);
        
        // For debugging - detailed console output
        ControllerRole role = static_cast<ControllerRole>(absoluteData.role);
        std::string prefix = (role == ControllerRole::Left) ? "left_vr" : "right_vr";
        
        // Print detailed info to console
        RCLCPP_INFO(this->get_logger(), "========== [%s] 控制器数据发布 ==========", prefix.c_str());
        RCLCPP_INFO(this->get_logger(), "测试方向: %s", current_test_direction_.c_str());
        RCLCPP_INFO(this->get_logger(), "VR坐标系 - 绝对位置: x=%.4f, y=%.4f, z=%.4f", 
                    absoluteData.pose_x, absoluteData.pose_y, absoluteData.pose_z);
        RCLCPP_INFO(this->get_logger(), "VR坐标系 - 相对位置: x=%.4f, y=%.4f, z=%.4f", 
                    relativeData.pose_x, relativeData.pose_y, relativeData.pose_z);
        RCLCPP_INFO(this->get_logger(), "ROS坐标系 - 绝对位置: x=%.4f, y=%.4f, z=%.4f", 
                    VRConfig::COORD_TRANSFORM_NEG * absoluteData.pose_z,
                    VRConfig::COORD_TRANSFORM_NEG * absoluteData.pose_x,
                    absoluteData.pose_y);
        RCLCPP_INFO(this->get_logger(), "==========================================\n");
    }

    // Helper method to get a topic name based on controller role
    std::string getRoleBasedTopicName(int role, const std::string& baseTopic) {
        std::string prefix = (role == 1) ? "left_vr" : "right_vr";
        return prefix + "/" + baseTopic;
    }

    int readFromSocket(char* buffer) {
        memset(buffer, 0, VRConfig::BUFFER_SIZE);
        return read(sock, buffer, VRConfig::BUFFER_SIZE);
    }

    bool parseControllerData(const std::string& receivedData) {
        try {
            json j = json::parse(receivedData);
            // Store JSON data to the struct
            jsonData.pose_x = j["pose"]["x"];
            jsonData.pose_y = j["pose"]["y"];
            jsonData.pose_z = j["pose"]["z"];
            jsonData.pose_qx = j["pose"]["qx"];
            jsonData.pose_qy = j["pose"]["qy"];
            jsonData.pose_qz = j["pose"]["qz"];
            jsonData.pose_qw = j["pose"]["qw"];
            jsonData.menu_button = j["buttons"]["menu"];
            jsonData.trigger_button = j["buttons"]["trigger"];
            jsonData.trackpad_touch = j["buttons"]["trackpad_touch"];
            jsonData.trackpad_button = j["buttons"]["trackpad_button"];
            jsonData.grip_button = j["buttons"]["grip"];
            jsonData.trackpad_x = j["trackpad"]["x"];
            jsonData.trackpad_y = j["trackpad"]["y"];
            jsonData.trigger = j["trigger"];
            jsonData.role = j["role"];
            jsonData.time = j["time"];
            return true;
        } catch (json::parse_error& e) {
            RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
            return false;
        }
    }

    void handleTriggerButton(const VRControllerData& filteredPose, VRControllerData& relativePose) {
        // Get references to the correct variables based on controller role
        VRControllerData& initialPose = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? initialPoseLeft : initialPoseRight;
        bool& triggerButtonPressed = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? triggerButtonPressedLeft : triggerButtonPressedRight;

        // Handle trigger button state
        if (filteredPose.trigger_button && !triggerButtonPressed) {
            // Trigger button just pressed
            initialPose = filteredPose;
            triggerButtonPressed = true;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger pressed - setting initial pose",
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");

            // 开始方向测试记录 (如果当前有选择测试方向)
            if (current_test_direction_ != "NONE") {
                startDirectionTest(filteredPose);
            }
        } else if (!filteredPose.trigger_button && triggerButtonPressed) {
            triggerButtonPressed = false;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Trigger released",
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");

            // 结束方向测试记录
            endDirectionTest();
        }

        if (triggerButtonPressed) {
            // Calculate and publish relative transform
            relativePose = calculateRelativePose(initialPose, filteredPose);
            publishTransform(relativePose, true);   // isRelative = true
            publishTransform(filteredPose);         // Publish absolute transform as well

            // 更新方向测试数据
            updateDirectionTest(filteredPose);
        } else {
            // Publish the absolute transform
            publishTransform(filteredPose);
        }
    }

    void handleMenuButton(const VRControllerData& filteredPose) {
        // If menu button is pressed, reset the initial pose
        if (filteredPose.menu_button) {
            VRControllerData& initialPose = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? initialPoseLeft : initialPoseRight;
            initialPose = filteredPose;
            RCLCPP_DEBUG(this->get_logger(), "[%s] Menu pressed - resetting initial pose", 
                        (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT");
        }
    }

    void handleGripButton(const VRControllerData& filteredPose) {
        // Handle direction switching with grip button
        bool& grip_was_pressed = (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? 
                                  grip_button_was_pressed_left_ : grip_button_was_pressed_right_;
        
        if (filteredPose.grip_button && !grip_was_pressed) {
            // Grip button just pressed - switch to next direction
            direction_index_ = (direction_index_ + 1) % test_directions_.size();
            current_test_direction_ = test_directions_[direction_index_];
            grip_was_pressed = true;
            
            RCLCPP_INFO(this->get_logger(), "\n");
            RCLCPP_INFO(this->get_logger(), "╔════════════════════════════════════════╗");
            RCLCPP_INFO(this->get_logger(), "║  测试方向已切换: %-20s ║", current_test_direction_.c_str());
            RCLCPP_INFO(this->get_logger(), "╚════════════════════════════════════════╝");
            RCLCPP_INFO(this->get_logger(), "\n");
            
            // Log direction change to file
            if (log_file_.is_open()) {
                log_file_ << "\n************************************************\n";
                log_file_ << "测试方向切换: " << current_test_direction_ << "\n";
                log_file_ << "************************************************\n\n";
                log_file_.flush();
            }
        } else if (!filteredPose.grip_button && grip_was_pressed) {
            grip_was_pressed = false;
        }
    }

    void printInstructions() {
        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "╔══════════════════════════════════════════════════════════════════╗");
        RCLCPP_INFO(this->get_logger(), "║            VR手柄平移测试 - 调试模式                             ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  坐标系说明 (ROS标准):                                           ║");
        RCLCPP_INFO(this->get_logger(), "║    X轴: 前(+) / 后(-)                                            ║");
        RCLCPP_INFO(this->get_logger(), "║    Y轴: 左(+) / 右(-)                                            ║");
        RCLCPP_INFO(this->get_logger(), "║    Z轴: 上(+) / 下(-)                                            ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  按键说明:                                                       ║");
        RCLCPP_INFO(this->get_logger(), "║  ▶ Grip按钮: 切换测试方向                                       ║");
        RCLCPP_INFO(this->get_logger(), "║  ▶ Trigger按钮: 按住开始记录，松开结束并显示摘要                ║");
        RCLCPP_INFO(this->get_logger(), "║  ▶ Menu按钮: 重置初始位置                                       ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  测试方向 (按Grip切换):                                          ║");
        RCLCPP_INFO(this->get_logger(), "║    NONE     - 不记录测试数据                                     ║");
        RCLCPP_INFO(this->get_logger(), "║    FORWARD  - 向前移动手柄 (期望: dx > 0)                        ║");
        RCLCPP_INFO(this->get_logger(), "║    BACKWARD - 向后移动手柄 (期望: dx < 0)                        ║");
        RCLCPP_INFO(this->get_logger(), "║    LEFT     - 向左移动手柄 (期望: dy > 0)                        ║");
        RCLCPP_INFO(this->get_logger(), "║    RIGHT    - 向右移动手柄 (期望: dy < 0)                        ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  测试步骤:                                                       ║");
        RCLCPP_INFO(this->get_logger(), "║  1. 按Grip按钮选择要测试的方向 (如 FORWARD)                      ║");
        RCLCPP_INFO(this->get_logger(), "║  2. 将手柄放在起始位置，按住Trigger按钮                         ║");
        RCLCPP_INFO(this->get_logger(), "║  3. 保持Trigger按下，向指定方向移动手柄约10-20cm                ║");
        RCLCPP_INFO(this->get_logger(), "║  4. 松开Trigger，查看测试摘要                                   ║");
        RCLCPP_INFO(this->get_logger(), "║  5. 重复步骤1-4测试其他方向                                     ║");
        RCLCPP_INFO(this->get_logger(), "╠══════════════════════════════════════════════════════════════════╣");
        RCLCPP_INFO(this->get_logger(), "║  话题: /controller_data                                          ║");
        RCLCPP_INFO(this->get_logger(), "║  日志文件: controller_debug_YYYYMMDD_HHMMSS.log                  ║");
        RCLCPP_INFO(this->get_logger(), "╚══════════════════════════════════════════════════════════════════╝");
        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), ">>> 当前测试方向: %s <<<", current_test_direction_.c_str());
        RCLCPP_INFO(this->get_logger(), "\n");
    }

    void logPerformanceMetrics(const VRControllerData& filteredPose, 
                              std::chrono::steady_clock::time_point start_time) {
        // Calculate and log processing time
        auto end_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        
        // Performance monitoring - logs every 100 messages to avoid overhead
        static int counter = 0;
        if (++counter % VRConfig::PERFORMANCE_LOG_INTERVAL == 0) {
            RCLCPP_INFO(this->get_logger(), "[%s] Processing time: %ld μs", 
                     (filteredPose.role == VRConfig::LEFT_CONTROLLER_ROLE) ? "LEFT" : "RIGHT", duration);
        }
    }

    void start() {
        while (sock < 0) {
            RCLCPP_INFO(this->get_logger(), "Attempting to connect to server...");
            establishServerConnection();
        }
        RCLCPP_INFO(this->get_logger(), "Connected to server.");
        
        // Print instructions
        printInstructions();

        // Main message processing loop - handles VR data reception and ROS publishing
        char buffer[VRConfig::BUFFER_SIZE] = {0};
        while (rclcpp::ok()) {
            int receivedByteCount = readFromSocket(buffer);
            if (receivedByteCount > 0) {
                // Measure processing time for performance monitoring
                auto start_time = std::chrono::steady_clock::now();
                
                std::string receivedData(buffer, receivedByteCount);
                if (!parseControllerData(receivedData)) {
                    continue; // Skip this iteration if parsing failed
                }

                // Log minimal information at INFO level
                ControllerRole role = static_cast<ControllerRole>(jsonData.role);
                std::string controller_name = (role == ControllerRole::Left) ? "LEFT" : "RIGHT";
                RCLCPP_INFO(this->get_logger(), "[%s] Received data at time: %s", 
                            controller_name.c_str(), jsonData.time.c_str());
                
                // Detailed logging only at DEBUG level - won't slow down processing in normal operation
                RCLCPP_DEBUG(this->get_logger(), "[%s] Pose: x=%f, y=%f, z=%f", 
                            controller_name.c_str(), jsonData.pose_x, jsonData.pose_y, jsonData.pose_z);
                RCLCPP_DEBUG(this->get_logger(), "[%s] Buttons: menu=%d, trigger=%d, trackpad=%d, grip=%d", 
                            controller_name.c_str(), 
                            jsonData.menu_button, jsonData.trigger_button, 
                            jsonData.trackpad_button, jsonData.grip_button);

                // Filter the pose data
                VRControllerData filteredPose = filterPose(jsonData);
                VRControllerData relativePose;

                // Handle grip button for direction switching
                handleGripButton(filteredPose);

                // Handle trigger button logic and transform publishing
                handleTriggerButton(filteredPose, relativePose);

                // Handle menu button logic
                handleMenuButton(filteredPose);

                // Publish controller data with role-based naming
                publishControllerData(filteredPose, relativePose);
                
                // Log performance metrics periodically
                logPerformanceMetrics(filteredPose, start_time);

            } else if (receivedByteCount == 0) {
                RCLCPP_WARN(this->get_logger(), "Connection closed by server. Attempting to reconnect...");
                reconnect();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Read error.");
                break;
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto client = std::make_shared<VRControllerClient>("127.0.0.1", 12345);
    client->start();
    rclcpp::spin(client);
    rclcpp::shutdown();
    return 0;
}
