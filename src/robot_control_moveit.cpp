#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>

class XArmMoveItControl : public rclcpp::Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;
    
    // Initial TCP state
    geometry_msgs::msg::Pose initial_tcp_pose_;
    
    // Control parameters
    static constexpr double POSITION_SCALE = 0.8;
    static constexpr double MAX_POSITION_CHANGE_PER_FRAME = 0.01; // 1cm
    static constexpr double MAX_ROTATION_PER_FRAME = 0.1; // ~5.7 degrees
    static constexpr double EMA_ALPHA = 0.7;
    
    // Smoothing variables
    bool use_ema_smoothing_;
    std::vector<double> smoothed_position_;
    double smoothed_rotation_;
    bool smoothing_initialized_;
    
    // Store move group name for deferred initialization
    std::string move_group_name_;

public:
    XArmMoveItControl(const std::string& move_group_name = "xarm6_experiment") 
        : Node("xarm_moveit_control"), use_ema_smoothing_(true), smoothed_rotation_(0.0), smoothing_initialized_(false), move_group_name_(move_group_name)
    {
        // Initialize smoothing arrays
        smoothed_position_.resize(3, 0.0);
        
        RCLCPP_INFO(this->get_logger(), "Node created, initializing MoveIt...");
    }
    
    bool initialize()
    {
        try {
            // Initialize MoveIt using the node (after construction is complete)
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), move_group_name_);
            planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
            
            // Set planning parameters
            move_group_->setPlanningTime(2.0);
            move_group_->setNumPlanningAttempts(5);
            move_group_->setMaxVelocityScalingFactor(0.3);
            move_group_->setMaxAccelerationScalingFactor(0.3);
            
            // SET THE CORRECT END EFFECTOR LINK TO MATCH TF
            move_group_->setEndEffectorLink("link_tcp");
            
            RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
            
            // Get initial TCP state
            getInitialTcpState();
            
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt: %s", e.what());
            return false;
        }
    }
    
    void getInitialTcpState()
    {
        initial_tcp_pose_ = move_group_->getCurrentPose().pose;
        
        RCLCPP_INFO(this->get_logger(), "Initial TCP position: [%.3f, %.3f, %.3f]", 
                 initial_tcp_pose_.position.x,
                 initial_tcp_pose_.position.y, 
                 initial_tcp_pose_.position.z);
        
        // Convert quaternion to RPY for display
        tf2::Quaternion q(initial_tcp_pose_.orientation.x,
                         initial_tcp_pose_.orientation.y,
                         initial_tcp_pose_.orientation.z,
                         initial_tcp_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        RCLCPP_INFO(this->get_logger(), "Initial TCP orientation (RPY): [%.3f, %.3f, %.3f] degrees", 
                 roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
    }
    
    geometry_msgs::msg::Pose getCurrentTcpPose()
    {
        return move_group_->getCurrentPose().pose;
    }
    
    bool setTcpPose(const geometry_msgs::msg::Pose& target_pose, bool execute = true)
    {
        move_group_->setPoseTarget(target_pose);
        
        if (execute) {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            
            move_group_->clearPoseTargets();
            return success;
        } else {
            // Just plan, don't execute
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            move_group_->clearPoseTargets();
            return success;
        }
    }
    
    geometry_msgs::msg::Pose mapControllerToTcp(const std::vector<double>& controller_pos_change, 
                                                double controller_y_rotation)
    {
        geometry_msgs::msg::Pose current_pose = getCurrentTcpPose();
        
        // SMOOTH POSITION MAPPING
        std::vector<double> raw_pos_change(3);
        for (int i = 0; i < 3; i++) {
            raw_pos_change[i] = controller_pos_change[i] * POSITION_SCALE;
        }
        
        // Apply tanh smoothing
        std::vector<double> tcp_pos_change(3);
        for (int i = 0; i < 3; i++) {
            tcp_pos_change[i] = std::tanh(raw_pos_change[i] / MAX_POSITION_CHANGE_PER_FRAME) * MAX_POSITION_CHANGE_PER_FRAME;
        }
        
        // Apply EMA smoothing
        if (use_ema_smoothing_) {
            if (!smoothing_initialized_) {
                smoothed_position_ = tcp_pos_change;
                smoothing_initialized_ = true;
            } else {
                for (int i = 0; i < 3; i++) {
                    smoothed_position_[i] = EMA_ALPHA * tcp_pos_change[i] + (1.0 - EMA_ALPHA) * smoothed_position_[i];
                }
            }
            tcp_pos_change = smoothed_position_;
        }
        
        // SMOOTH ROTATION MAPPING
        double mapped_rotation_change = std::tanh(controller_y_rotation / MAX_ROTATION_PER_FRAME) * MAX_ROTATION_PER_FRAME;
        
        // Apply EMA smoothing to rotation
        if (use_ema_smoothing_) {
            smoothed_rotation_ = EMA_ALPHA * mapped_rotation_change + (1.0 - EMA_ALPHA) * smoothed_rotation_;
            mapped_rotation_change = smoothed_rotation_;
        }
        
        // Create target pose
        geometry_msgs::msg::Pose target_pose = current_pose;
        
        // Apply position changes
        target_pose.position.x += tcp_pos_change[0];
        target_pose.position.y += tcp_pos_change[1];  
        target_pose.position.z += tcp_pos_change[2];
        
        // Apply rotation change to yaw (Z-axis)
        tf2::Quaternion current_q(current_pose.orientation.x,
                                 current_pose.orientation.y,
                                 current_pose.orientation.z,
                                 current_pose.orientation.w);
        
        tf2::Matrix3x3 current_m(current_q);
        double roll, pitch, yaw;
        current_m.getRPY(roll, pitch, yaw);
        
        // Add rotation change to yaw
        yaw += mapped_rotation_change;
        
        // Convert back to quaternion
        tf2::Quaternion target_q;
        target_q.setRPY(roll, pitch, yaw);
        target_q.normalize();
        
        target_pose.orientation.x = target_q.x();
        target_pose.orientation.y = target_q.y();
        target_pose.orientation.z = target_q.z();
        target_pose.orientation.w = target_q.w();
        
        // Debug output for large changes
        double pos_change_magnitude = std::sqrt(tcp_pos_change[0]*tcp_pos_change[0] + 
                                               tcp_pos_change[1]*tcp_pos_change[1] + 
                                               tcp_pos_change[2]*tcp_pos_change[2]);
        
        if (pos_change_magnitude > 0.001) { // > 1mm change
            RCLCPP_INFO(this->get_logger(), 
                       "\n[POS CHANGE] Controller: [%.1f, %.1f, %.1f]mm -> TCP: [%.1f, %.1f, %.1f]mm", 
                       controller_pos_change[0]*1000, controller_pos_change[1]*1000, controller_pos_change[2]*1000,
                       tcp_pos_change[0]*1000, tcp_pos_change[1]*1000, tcp_pos_change[2]*1000);
        }
        
        if (std::abs(mapped_rotation_change) > 0.001) { // > 0.057 degrees
            RCLCPP_INFO(this->get_logger(), 
                       "\n[ROTATION] Controller Y: %.3f° -> TCP Z: %.3f°",
                       controller_y_rotation * 180.0/M_PI,
                       mapped_rotation_change * 180.0/M_PI);
        }
        
        return target_pose;
    }
};

// Mock VR interface for testing (replace with actual VR integration)
class MockVRInterface 
{
private:
    std::vector<double> prev_position_;
    double prev_rotation_;
    bool initialized_;
    rclcpp::Clock::SharedPtr clock_;
    
public:
    MockVRInterface(rclcpp::Node::SharedPtr node) : prev_position_(3, 0.0), prev_rotation_(0.0), initialized_(false) 
    {
        clock_ = node->get_clock();
    }
    
    std::pair<std::vector<double>, double> getControllerChange()
    {
        // Mock controller data - much smaller movements to stay within workspace
        double time_sec = clock_->now().seconds();
        std::vector<double> current_pos = {0.01 * std::sin(time_sec), 
                                          0.01 * std::cos(time_sec), 
                                          0.005 * std::sin(time_sec * 0.5)};
        double current_rot = 0.01 * std::sin(time_sec * 2.0);
        
        if (!initialized_) {
            prev_position_ = current_pos;
            prev_rotation_ = current_rot;
            initialized_ = true;
            return {{0.0, 0.0, 0.0}, 0.0};
        }
        
        std::vector<double> pos_change(3);
        for (int i = 0; i < 3; i++) {
            pos_change[i] = current_pos[i] - prev_position_[i];
        }
        double rot_change = current_rot - prev_rotation_;
        
        prev_position_ = current_pos;
        prev_rotation_ = current_rot;
        
        return {pos_change, rot_change};
    }
    
    bool shouldExit()
    {
        // Mock exit condition - replace with actual VR menu button
        return false;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<XArmMoveItControl>();
        
        // Initialize MoveIt after node construction is complete
        if (!node->initialize()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize MoveIt interface");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Initializing VR interface...");
        MockVRInterface vr_interface(node); // Replace with actual VR interface
        
        RCLCPP_INFO(node->get_logger(), "Starting teleoperation loop...");
        RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit");
        
        rclcpp::Rate loop_rate(10); // 10 Hz
        int loop_count = 0;
        
        while (rclcpp::ok()) {
            auto [pos_change, rot_change] = vr_interface.getControllerChange();
            
            if (vr_interface.shouldExit()) {
                RCLCPP_INFO(node->get_logger(), "Exit requested");
                break;
            }
            
            // Only start moving after a few seconds to let everything settle
            if (loop_count < 30) {
                RCLCPP_INFO(node->get_logger(), "Waiting... loop %d/30", loop_count);
                rclcpp::spin_some(node);
                loop_count++;
                loop_rate.sleep();
                continue;
            }
            
            // Map controller changes to TCP pose
            geometry_msgs::msg::Pose target_pose = node->mapControllerToTcp(pos_change, rot_change);
            
            // Execute motion
            bool success = node->setTcpPose(target_pose, true);
            
            if (!success && loop_count % 10 == 0) {
                RCLCPP_WARN(node->get_logger(), "Failed to plan/execute motion at loop %d", loop_count);
            }
            
            // Spin once to handle ROS callbacks
            rclcpp::spin_some(node);
            
            loop_count++;
            loop_rate.sleep();
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "Shutting down...");
    rclcpp::shutdown();
    return 0;
}