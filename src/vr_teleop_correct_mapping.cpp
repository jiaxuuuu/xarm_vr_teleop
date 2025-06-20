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
#include <cstdio>
#include <memory>
#include <string>
#include <array>
#include <stdexcept>
#include <iostream>
#include <sstream>

class CorrectMappingVRTeleop
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    
    // TF for getting current TCP pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Previous controller state for frame-to-frame tracking
    std::vector<double> prev_controller_position_;
    double prev_controller_y_rotation_;
    bool controller_initialized_;
    
    // Control parameters (from original)
    static constexpr double POSITION_SCALE = 0.8;
    static constexpr double MAX_POSITION_CHANGE_PER_FRAME = 0.01; // 1cm
    static constexpr double MAX_ROTATION_PER_FRAME = 0.1; // ~5.7 degrees
    static constexpr double EMA_ALPHA = 0.7;
    
    // Smoothing variables
    std::vector<double> smoothed_position_;
    double smoothed_rotation_;
    bool smoothing_initialized_;

public:
    CorrectMappingVRTeleop(rclcpp::Node::SharedPtr node) : node_(node),
                                                           controller_initialized_(false),
                                                           smoothing_initialized_(false),
                                                           smoothed_rotation_(0.0)
    {
        RCLCPP_INFO(node_->get_logger(), "=== Correct Controller Mapping VR Teleop ===");
        
        // Initialize arrays
        prev_controller_position_.resize(3, 0.0);
        smoothed_position_.resize(3, 0.0);
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create move group interface (for planning/execution only)
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "xarm6_experiment");
            
        move_group_->setPlanningTime(2.0);
        move_group_->setNumPlanningAttempts(5);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        
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
                // Fallback to link6 if link_tcp doesn't exist
                try {
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                        "world", "link6", tf2::TimePointZero, tf2::durationFromSec(1.0));
                    
                    pose.position.x = transform.transform.translation.x;
                    pose.position.y = transform.transform.translation.y;
                    pose.position.z = transform.transform.translation.z;
                    pose.orientation = transform.transform.rotation;
                    
                    if (!frame_logged) {
                        RCLCPP_WARN(node_->get_logger(), "⚠️  FALLBACK: Using world -> link6 (link_tcp not available)");
                        frame_logged = true;
                    }
                    return pose;
                    
                } catch (const tf2::TransformException& ex3) {
                    try {
                        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                            "base_link", "link6", tf2::TimePointZero, tf2::durationFromSec(1.0));
                        
                        pose.position.x = transform.transform.translation.x;
                        pose.position.y = transform.transform.translation.y;
                        pose.position.z = transform.transform.translation.z;
                        pose.orientation = transform.transform.rotation;
                        
                        if (!frame_logged) {
                            RCLCPP_WARN(node_->get_logger(), "⚠️  FALLBACK: Using base_link -> link6 (link_tcp not available)");
                            frame_logged = true;
                        }
                        return pose;
                        
                    } catch (const tf2::TransformException& ex4) {
                        RCLCPP_ERROR(node_->get_logger(), "❌ ALL TF lookups failed: %s", ex4.what());
                        pose.position.x = pose.position.y = pose.position.z = 0.0;
                        pose.orientation.w = 1.0;
                        pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
                        return pose;
                    }
                }
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
    
    std::pair<std::vector<double>, double> processControllerInput(
        const std::vector<double>& current_controller_pos,
        double current_controller_y_rotation)
    {
        // FRAME-TO-FRAME TRACKING (like original)
        std::vector<double> position_change(3, 0.0);
        double rotation_change = 0.0;
        
        if (!controller_initialized_) {
            // First frame - no change
            prev_controller_position_ = current_controller_pos;
            prev_controller_y_rotation_ = current_controller_y_rotation;
            controller_initialized_ = true;
            return {position_change, rotation_change};
        }
        
        // Calculate frame-to-frame changes
        for (int i = 0; i < 3; i++) {
            position_change[i] = current_controller_pos[i] - prev_controller_position_[i];
        }
        rotation_change = current_controller_y_rotation - prev_controller_y_rotation_;
        
        // DEBUG: Show frame-to-frame changes
        static int frame_debug_count = 0;
        if (frame_debug_count++ % 25 == 0 && (std::abs(position_change[0]) > 0.001 || std::abs(position_change[1]) > 0.001 || std::abs(position_change[2]) > 0.001)) {
            printf("FRAME-TO-FRAME DEBUG: pos_change=[%.4f,%.4f,%.4f], rot_change=%.3f°\n",
                   position_change[0], position_change[1], position_change[2],
                   rotation_change * 180.0/M_PI);
        }
        
        // Update previous state
        prev_controller_position_ = current_controller_pos;
        prev_controller_y_rotation_ = current_controller_y_rotation;
        
        return {position_change, rotation_change};
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
        
        // DEBUG: Show detailed coordinate transformation
        RCLCPP_INFO(node_->get_logger(), 
                   "COORD DEBUG: current=[%.3f,%.3f,%.3f] + change=[%.3f,%.3f,%.3f] = target=[%.3f,%.3f,%.3f]",
                   current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z,
                   tcp_pos_change[0], tcp_pos_change[1], tcp_pos_change[2],
                   target_pose.position.x, target_pose.position.y, target_pose.position.z);
        
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

// Real VR Controller Interface using Python bridge
class RealVRControllerInterface 
{
private:
    rclcpp::Node::SharedPtr node_;
    FILE* vr_process_;
    bool initialized_;
    
    std::string exec_command(const char* cmd) {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe) {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }
        return result;
    }
    
public:
    RealVRControllerInterface(rclcpp::Node::SharedPtr node) : node_(node), vr_process_(nullptr), initialized_(false)
    {
        RCLCPP_INFO(node_->get_logger(), "🎮 Initializing REAL VR Controller Interface");
        RCLCPP_INFO(node_->get_logger(), "📡 Starting VR bridge process...");
        
        // Start the Python VR bridge process with proper working directory
        std::string vr_script_path = "/home/jiaxu/xarm_vr_teleop/scripts/simple_vr_bridge.py";
        std::string command = "cd /home/jiaxu/xarm_vr_teleop && python3 " + vr_script_path;
        
        RCLCPP_INFO(node_->get_logger(), "🔧 Running command: %s", command.c_str());
        
        vr_process_ = popen(command.c_str(), "r");
        if (!vr_process_) {
            RCLCPP_ERROR(node_->get_logger(), "❌ Failed to start VR bridge process");
            RCLCPP_ERROR(node_->get_logger(), "📋 Command was: %s", command.c_str());
            return;
        }
        
        // Read initialization response with timeout
        char buffer[1024];
        RCLCPP_INFO(node_->get_logger(), "⏳ Waiting for VR bridge initialization...");
        
        if (fgets(buffer, sizeof(buffer), vr_process_)) {
            std::string response(buffer);
            RCLCPP_INFO(node_->get_logger(), "📥 VR bridge response: %s", response.c_str());
            
            if (response.find("initialized") != std::string::npos) {
                initialized_ = true;
                RCLCPP_INFO(node_->get_logger(), "✅ VR Controller initialized successfully!");
                RCLCPP_INFO(node_->get_logger(), "🎯 Using HTC Vive Pro in VR world coordinate system");
                RCLCPP_INFO(node_->get_logger(), "🔄 VR world frame = robot base_link frame");
            } else if (response.find("error") != std::string::npos) {
                RCLCPP_ERROR(node_->get_logger(), "❌ VR initialization failed: %s", response.c_str());
                RCLCPP_ERROR(node_->get_logger(), "🚨 Make sure HTC Vive is connected and SteamVR is running!");
            } else {
                RCLCPP_ERROR(node_->get_logger(), "❌ Unexpected VR response: %s", response.c_str());
            }
        } else {
            RCLCPP_ERROR(node_->get_logger(), "❌ No response from VR bridge - process may have failed");
            RCLCPP_ERROR(node_->get_logger(), "🚨 Check that HTC Vive hardware is connected!");
        }
    }
    
    ~RealVRControllerInterface() {
        if (vr_process_) {
            fprintf(vr_process_, "exit\n");
            fflush(vr_process_);
            pclose(vr_process_);
        }
    }
    
    std::pair<std::vector<double>, double> getCurrentControllerState()
    {
        if (!initialized_ || !vr_process_) {
            return {{0.0, 0.0, 0.0}, 0.0};
        }
        
        try {
            // Send command to get controller state
            fprintf(vr_process_, "get_state\n");
            fflush(vr_process_);
            
            // Read response
            char buffer[1024];
            if (fgets(buffer, sizeof(buffer), vr_process_)) {
                std::string response(buffer);
                
                // Parse JSON response (simple parsing)
                if (response.find("error") != std::string::npos) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "VR read error: %s", response.c_str());
                    return {{0.0, 0.0, 0.0}, 0.0};
                }
                
                // Extract frame-to-frame changes from JSON
                // Parse "position_change": [x, y, z] and "y_rotation_change": value
                std::vector<double> position_change(3, 0.0);
                double y_rotation_change = 0.0;
                
                // Find position_change array
                size_t pos_start = response.find("\"position_change\": [");
                if (pos_start != std::string::npos) {
                    pos_start += 20; // Skip '"position_change": ['
                    size_t pos_end = response.find("]", pos_start);
                    if (pos_end != std::string::npos) {
                        std::string pos_str = response.substr(pos_start, pos_end - pos_start);
                        std::stringstream ss(pos_str);
                        std::string item;
                        int i = 0;
                        while (std::getline(ss, item, ',') && i < 3) {
                            position_change[i] = std::stod(item);
                            i++;
                        }
                    }
                }
                
                // Find y_rotation_change
                size_t rot_start = response.find("\"y_rotation_change\": ");
                if (rot_start != std::string::npos) {
                    rot_start += 21; // Skip '"y_rotation_change": '
                    size_t rot_end = response.find_first_of(",}", rot_start);
                    if (rot_end != std::string::npos) {
                        std::string rot_str = response.substr(rot_start, rot_end - rot_start);
                        y_rotation_change = std::stod(rot_str);
                    }
                }
                
                // Debug output occasionally
                static int debug_count = 0;
                if (debug_count++ % 50 == 0 && (std::abs(position_change[0]) > 0.001 || std::abs(position_change[1]) > 0.001 || std::abs(position_change[2]) > 0.001)) {
                    RCLCPP_INFO(node_->get_logger(), "🎮 VR Controller change: pos=[%.4f,%.4f,%.4f], y_rot_change=%.3f°",
                               position_change[0], position_change[1], position_change[2], y_rotation_change * 180.0/M_PI);
                }
                
                return {position_change, y_rotation_change};
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "VR read exception: %s", e.what());
        }
        
        return {{0.0, 0.0, 0.0}, 0.0};
    }
    
    bool isInitialized() {
        return initialized_ && vr_process_;
    }
    
    bool shouldExit() {
        // Check if VR process is still running and if exit was requested
        if (!initialized_ || !vr_process_) {
            return true;
        }
        
        // For now, just return false - could implement controller button checking later
        return false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("correct_mapping_vr_teleop");
    
    try {
        CorrectMappingVRTeleop teleop(node);
        RealVRControllerInterface vr_controller(node);
        
        // SAFETY CHECK: Ensure VR is properly initialized before proceeding
        if (!vr_controller.isInitialized()) {
            RCLCPP_ERROR(node->get_logger(), "🚨 VR Controller NOT initialized - ABORTING for safety!");
            RCLCPP_ERROR(node->get_logger(), "📋 Checklist:");
            RCLCPP_ERROR(node->get_logger(), "   1. Connect HTC Vive headset and controllers");
            RCLCPP_ERROR(node->get_logger(), "   2. Start SteamVR");
            RCLCPP_ERROR(node->get_logger(), "   3. Ensure controller is tracked (green in SteamVR)");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Getting initial TCP pose from TF...");
        
        // Get initial pose from TF
        auto current_tcp_pose = teleop.getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node->get_logger(), "Initial TCP: [%.3f, %.3f, %.3f]",
                   current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z);
        
        if (current_tcp_pose.position.x == 0.0 && current_tcp_pose.position.y == 0.0 && current_tcp_pose.position.z == 0.0) {
            RCLCPP_WARN(node->get_logger(), "Got zero pose - TF might not be available");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Starting correct mapping teleop loop...");
        
        rclcpp::Rate loop_rate(5); // 5 Hz for smooth control
        int loop_count = 0;
        
        // Store initial pose for comparison
        auto initial_tcp_pose = current_tcp_pose;
        RCLCPP_INFO(node->get_logger(), "=== Starting from initial TCP pose ===");
        
        while (rclcpp::ok() && loop_count < 50) { // Extended test
            // Get frame-to-frame controller changes directly from VR bridge
            auto [pos_change, rot_change] = vr_controller.getCurrentControllerState();
            
            // SAFETY CHECK: If VR bridge failed, don't move the robot!
            if (vr_controller.shouldExit() || !vr_controller.isInitialized()) {
                RCLCPP_ERROR(node->get_logger(), "❌ VR system not available - STOPPING for safety!");
                break;
            }
            
            // SAFETY CHECK: Verify changes are reasonable (< 5cm per frame)
            double change_magnitude = std::sqrt(pos_change[0]*pos_change[0] + pos_change[1]*pos_change[1] + pos_change[2]*pos_change[2]);
            if (change_magnitude > 0.05) {  // 5cm safety limit
                RCLCPP_WARN(node->get_logger(), "⚠️  Large movement detected (%.3fm) - skipping for safety", change_magnitude);
                continue;
            }
            
            // Get current TCP pose from TF (this is the CURRENT pose, not initial)
            current_tcp_pose = teleop.getCurrentTcpPoseFromTF();
            
            // Apply correct controller mapping (relative to CURRENT pose)
            // pos_change and rot_change are already frame-to-frame differences
            auto target_pose = teleop.applyControllerMapping(current_tcp_pose, pos_change, rot_change);
            
            // Show relationship to initial pose
            if (loop_count % 5 == 0) {
                auto delta_from_initial = std::sqrt(
                    std::pow(current_tcp_pose.position.x - initial_tcp_pose.position.x, 2) +
                    std::pow(current_tcp_pose.position.y - initial_tcp_pose.position.y, 2) +
                    std::pow(current_tcp_pose.position.z - initial_tcp_pose.position.z, 2)
                );
                
                RCLCPP_INFO(node->get_logger(), 
                           "[%d] Current TCP: [%.3f,%.3f,%.3f] | Δ from initial: %.3fm | pos_change: [%.1f,%.1f,%.1f]mm",
                           loop_count,
                           current_tcp_pose.position.x, current_tcp_pose.position.y, current_tcp_pose.position.z,
                           delta_from_initial,
                           pos_change[0]*1000, pos_change[1]*1000, pos_change[2]*1000);
            }
            
            // Execute motion
            bool success = teleop.moveToPose(target_pose);
            
            if (!success) {
                RCLCPP_WARN(node->get_logger(), "[%d] Motion FAILED", loop_count);
            }
            
            rclcpp::spin_some(node);
            loop_count++;
            loop_rate.sleep();
        }
        
        RCLCPP_INFO(node->get_logger(), "Correct mapping test complete!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}