#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <iostream>
#include <sstream>
#include <string>

class CorrectMappingVRTeleop
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    
    // TF for getting current TCP pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Control parameters (increased for better responsiveness)
    static constexpr double POSITION_SCALE = 5.0; // Increased from 0.8 to 5.0!
    static constexpr double MAX_POSITION_CHANGE_PER_FRAME = 0.05; // 5cm (increased from 1cm)
    static constexpr double MAX_ROTATION_PER_FRAME = 0.3; // ~17 degrees (increased)
    static constexpr double EMA_ALPHA = 0.9; // Less smoothing, more responsive
    
    // Smoothing variables
    std::vector<double> smoothed_position_;
    double smoothed_rotation_;
    bool smoothing_initialized_;

public:
    CorrectMappingVRTeleop(rclcpp::Node::SharedPtr node) : node_(node),
                                                           smoothing_initialized_(false),
                                                           smoothed_rotation_(0.0)
    {
        RCLCPP_INFO(node_->get_logger(), "=== Manual Bridge VR Teleop ===");
        
        // Initialize arrays
        smoothed_position_.resize(3, 0.0);
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create move group interface (for planning/execution only)
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "xarm6_experiment");
            
        move_group_->setPlanningTime(0.5);  // Reduced from 2.0s to 0.5s
        move_group_->setNumPlanningAttempts(2);  // Reduced from 5 to 2
        move_group_->setMaxVelocityScalingFactor(0.8);  // Increased for faster movement
        move_group_->setMaxAccelerationScalingFactor(0.8);
        
        // SET THE CORRECT END EFFECTOR LINK TO MATCH TF
        move_group_->setEndEffectorLink("link_tcp");
        
        RCLCPP_INFO(node_->get_logger(), "MoveIt initialized for planning/execution");
        RCLCPP_INFO(node_->get_logger(), "End effector set to: %s", move_group_->getEndEffectorLink().c_str());
        
        // Wait for TF to be available
        RCLCPP_INFO(node_->get_logger(), "Waiting for TF...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        RCLCPP_INFO(node_->get_logger(), "Coordinate mapping:");
        RCLCPP_INFO(node_->get_logger(), "- VR world frame = robot base_link frame");
        RCLCPP_INFO(node_->get_logger(), "- Controller Y-axis rotation -> TCP Z-axis rotation (yaw)");
        RCLCPP_INFO(node_->get_logger(), "- Frame-to-frame tracking for smooth control");
    }
    
    geometry_msgs::msg::Pose getCurrentTcpPoseFromTF()
    {
        geometry_msgs::msg::Pose pose;
        static bool frame_logged = false;
        
        try {
            // Get transform from world/base_link to link_tcp (TCP)
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "world", "link_tcp", tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;
            
            if (!frame_logged) {
                RCLCPP_INFO(node_->get_logger(), "✅ Using TF: world -> link_tcp");
                frame_logged = true;
            }
            return pose;
            
        } catch (const tf2::TransformException& ex) {
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "base_link", "link_tcp", tf2::TimePointZero, tf2::durationFromSec(1.0));
                
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                pose.orientation = transform.transform.rotation;
                
                if (!frame_logged) {
                    RCLCPP_INFO(node_->get_logger(), "✅ Using TF: base_link -> link_tcp");
                    frame_logged = true;
                }
                return pose;
                
            } catch (const tf2::TransformException& ex2) {
                RCLCPP_ERROR(node_->get_logger(), "❌ ALL TF lookups failed: %s", ex2.what());
                pose.position.x = pose.position.y = pose.position.z = 0.0;
                pose.orientation.w = 1.0;
                pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
                return pose;
            }
        }
    }
    
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            move_group_->setPoseTarget(target_pose);
            
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            
            move_group_->clearPoseTargets();
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Motion error: %s", e.what());
            return false;
        }
    }
    
    geometry_msgs::msg::Pose applyControllerMapping(
        const geometry_msgs::msg::Pose& current_tcp_pose,
        const std::vector<double>& controller_pos_change,
        double controller_y_rotation_change)
    {
        // SMOOTH POSITION MAPPING (like original)
        std::vector<double> raw_pos_change(3);
        for (int i = 0; i < 3; i++) {
            raw_pos_change[i] = controller_pos_change[i] * POSITION_SCALE;
        }
        
        // Apply tanh smoothing to compress large movements while preserving small ones
        std::vector<double> tcp_pos_change(3);
        for (int i = 0; i < 3; i++) {
            tcp_pos_change[i] = std::tanh(raw_pos_change[i] / MAX_POSITION_CHANGE_PER_FRAME) * MAX_POSITION_CHANGE_PER_FRAME;
        }
        
        // Apply EMA smoothing if enabled
        if (!smoothing_initialized_) {
            smoothed_position_ = tcp_pos_change;
            smoothing_initialized_ = true;
        } else {
            for (int i = 0; i < 3; i++) {
                smoothed_position_[i] = EMA_ALPHA * tcp_pos_change[i] + (1.0 - EMA_ALPHA) * smoothed_position_[i];
            }
        }
        tcp_pos_change = smoothed_position_;
        
        // SMOOTH ROTATION MAPPING (like original)
        // Controller Y-axis rotation -> TCP Z-axis rotation (yaw)
        double raw_rotation_change = controller_y_rotation_change;
        double mapped_rotation_change = std::tanh(raw_rotation_change / MAX_ROTATION_PER_FRAME) * MAX_ROTATION_PER_FRAME;
        
        // Apply EMA smoothing to rotation
        smoothed_rotation_ = EMA_ALPHA * mapped_rotation_change + (1.0 - EMA_ALPHA) * smoothed_rotation_;
        mapped_rotation_change = smoothed_rotation_;
        
        // Create target pose
        geometry_msgs::msg::Pose target_pose = current_tcp_pose;
        
        // Apply position changes in base_link frame (VR world frame = base_link frame)
        target_pose.position.x += tcp_pos_change[0];
        target_pose.position.y += tcp_pos_change[1];
        target_pose.position.z += tcp_pos_change[2];
        
        // Apply rotation change to TCP Z-axis (yaw)
        tf2::Quaternion current_q(
            current_tcp_pose.orientation.x,
            current_tcp_pose.orientation.y,
            current_tcp_pose.orientation.z,
            current_tcp_pose.orientation.w);
            
        tf2::Matrix3x3 current_m(current_q);
        double roll, pitch, yaw;
        current_m.getRPY(roll, pitch, yaw);
        
        // Add controller Y-rotation change to TCP yaw
        yaw += mapped_rotation_change;
        
        tf2::Quaternion target_q;
        target_q.setRPY(roll, pitch, yaw);
        target_q.normalize();
        
        target_pose.orientation.x = target_q.x();
        target_pose.orientation.y = target_q.y();
        target_pose.orientation.z = target_q.z();
        target_pose.orientation.w = target_q.w();
        
        // Debug output for large changes (like original)
        double pos_change_magnitude = std::sqrt(tcp_pos_change[0]*tcp_pos_change[0] + 
                                               tcp_pos_change[1]*tcp_pos_change[1] + 
                                               tcp_pos_change[2]*tcp_pos_change[2]);
        
        if (pos_change_magnitude > 0.001) { // > 1mm change
            RCLCPP_INFO(node_->get_logger(), 
                       "[POS] Controller: [%.1f, %.1f, %.1f]mm -> TCP: [%.1f, %.1f, %.1f]mm", 
                       controller_pos_change[0]*1000, controller_pos_change[1]*1000, controller_pos_change[2]*1000,
                       tcp_pos_change[0]*1000, tcp_pos_change[1]*1000, tcp_pos_change[2]*1000);
        }
        
        if (std::abs(mapped_rotation_change) > 0.001) { // > 0.057 degrees
            RCLCPP_INFO(node_->get_logger(), 
                       "[ROT] Controller Y: %.3f° -> TCP Z(yaw): %.3f°",
                       controller_y_rotation_change * 180.0/M_PI,
                       mapped_rotation_change * 180.0/M_PI);
        }
        
        return target_pose;
    }
};

// Simple function to parse JSON from manual VR bridge
std::pair<std::vector<double>, double> parseVRData(const std::string& json_line)
{
    // Simple JSON parsing for: {"position_change": [x, y, z], "y_rotation_change": value}
    std::vector<double> pos_change(3, 0.0);
    double rot_change = 0.0;
    
    try {
        // Find position_change array
        size_t pos_start = json_line.find("\"position_change\": [");
        if (pos_start != std::string::npos) {
            pos_start += 20; // Skip '"position_change": ['
            size_t pos_end = json_line.find("]", pos_start);
            if (pos_end != std::string::npos) {
                std::string pos_str = json_line.substr(pos_start, pos_end - pos_start);
                std::stringstream ss(pos_str);
                std::string item;
                int i = 0;
                while (std::getline(ss, item, ',') && i < 3) {
                    pos_change[i] = std::stod(item);
                    i++;
                }
            }
        }
        
        // Find y_rotation_change
        size_t rot_start = json_line.find("\"y_rotation_change\": ");
        if (rot_start != std::string::npos) {
            rot_start += 21; // Skip '"y_rotation_change": '
            size_t rot_end = json_line.find_first_of(",}", rot_start);
            if (rot_end != std::string::npos) {
                std::string rot_str = json_line.substr(rot_start, rot_end - rot_start);
                rot_change = std::stod(rot_str);
            }
        }
    } catch (const std::exception& e) {
        // Return zeros on parse error
    }
    
    return {pos_change, rot_change};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("manual_bridge_vr_teleop");
    
    try {
        CorrectMappingVRTeleop teleop(node);
        
        RCLCPP_INFO(node->get_logger(), "Getting initial TCP pose from TF...");
        
        // Get initial pose from TF
        auto current_tcp_pose = teleop.getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node->get_logger(), "Initial TCP: [%.3f, %.3f, %.3f]",
                   current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z);
        
        if (current_tcp_pose.position.x == 0.0 && current_tcp_pose.position.y == 0.0 && current_tcp_pose.position.z == 0.0) {
            RCLCPP_WARN(node->get_logger(), "Got zero pose - TF might not be available");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "🎮 Ready to receive VR data from manual bridge!");
        RCLCPP_INFO(node->get_logger(), "📋 Instructions:");
        RCLCPP_INFO(node->get_logger(), "   1. Run: python3 scripts/manual_vr_bridge.py | ros2 run xarm_vr_teleop vr_teleop_manual_bridge");
        RCLCPP_INFO(node->get_logger(), "   2. Or pipe VR data to this program");
        
        std::string line;
        int frame_count = 0;
        
        // Read VR data from stdin (piped from manual bridge)
        while (rclcpp::ok() && std::getline(std::cin, line)) {
            if (line.empty()) continue;
            
            // Parse VR data
            auto [pos_change, rot_change] = parseVRData(line);
            
            // SAFETY CHECK: Verify changes are reasonable (< 2cm per frame)
            double change_magnitude = std::sqrt(pos_change[0]*pos_change[0] + pos_change[1]*pos_change[1] + pos_change[2]*pos_change[2]);
            if (change_magnitude > 0.02) {  // 2cm safety limit (reduced from 5cm)
                RCLCPP_WARN(node->get_logger(), "⚠️  Large movement detected (%.3fm) - skipping for safety", change_magnitude);
                continue;
            }
            
            // DEBUG: Show all movements > 0.1mm
            if (change_magnitude > 0.0001) {
                RCLCPP_INFO(node->get_logger(), "🎮 VR input: [%.2f,%.2f,%.2f]mm, mag=%.2fmm", 
                           pos_change[0]*1000, pos_change[1]*1000, pos_change[2]*1000, change_magnitude*1000);
            }
            
            // Get current TCP pose from TF
            current_tcp_pose = teleop.getCurrentTcpPoseFromTF();
            
            // Apply correct controller mapping
            auto target_pose = teleop.applyControllerMapping(current_tcp_pose, pos_change, rot_change);
            
            // Show progress occasionally
            if (frame_count % 20 == 0) {
                RCLCPP_INFO(node->get_logger(), 
                           "[%d] TCP: [%.3f,%.3f,%.3f] | change: [%.1f,%.1f,%.1f]mm",
                           frame_count,
                           current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z,
                           pos_change[0]*1000, pos_change[1]*1000, pos_change[2]*1000);
            }
            
            // Execute motion (only if there's significant change)
            if (change_magnitude > 0.0002) {  // > 0.2mm (reduced threshold for more movements)
                bool success = teleop.moveToPose(target_pose);
                if (!success) {
                    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 3000, 
                                         "❌ Motion FAILED for change: %.2fmm", change_magnitude*1000);
                } else {
                    RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
                                         "✅ Motion SUCCESS for change: %.2fmm", change_magnitude*1000);
                }
            }
            
            rclcpp::spin_some(node);
            frame_count++;
        }
        
        RCLCPP_INFO(node->get_logger(), "Manual bridge VR teleop complete!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}