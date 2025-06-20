#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <memory>

class DirectVRTeleop : public rclcpp::Node
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // Current robot state
    std::vector<double> current_joint_positions_;
    geometry_msgs::msg::Pose current_tcp_pose_;
    bool robot_state_received_;
    
    // Robot model for FK/IK
    robot_model_loader::RobotModelLoader robot_model_loader_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    
    // Control parameters
    static constexpr double POSITION_SCALE = 0.8;
    static constexpr double MAX_POSITION_CHANGE_PER_FRAME = 0.01;
    static constexpr double MAX_ROTATION_PER_FRAME = 0.1;
    static constexpr double EMA_ALPHA = 0.7;
    
    // Smoothing variables
    std::vector<double> smoothed_position_;
    double smoothed_rotation_;
    bool smoothing_initialized_;

public:
    DirectVRTeleop() : Node("direct_vr_teleop"), 
                       robot_state_received_(false),
                       smoothed_rotation_(0.0), 
                       smoothing_initialized_(false),
                       robot_model_loader_(shared_from_this())
    {
        RCLCPP_INFO(this->get_logger(), "=== Direct VR Teleop (No State Monitor) ===");
        
        // Initialize arrays
        current_joint_positions_.resize(6, 0.0);
        smoothed_position_.resize(3, 0.0);
        
        // Get robot model
        robot_model_ = robot_model_loader_.getModel();
        if (!robot_model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
            return;
        }
        
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
        robot_state_->setToDefaultValues();
        
        RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully");
        
        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DirectVRTeleop::jointStateCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Waiting for robot state...");
    }
    
    bool initialize()
    {
        // Wait for first joint state
        while (!robot_state_received_ && rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        if (!robot_state_received_) {
            RCLCPP_ERROR(this->get_logger(), "No joint states received");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Initial joint state received");
        
        // Initialize MoveIt (for planning only, not state monitoring)
        try {
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), "xarm6_experiment");
                
            move_group_->setPlanningTime(2.0);
            move_group_->setNumPlanningAttempts(3);
            move_group_->setMaxVelocityScalingFactor(0.3);
            move_group_->setMaxAccelerationScalingFactor(0.3);
            
            // SET THE CORRECT END EFFECTOR LINK TO MATCH TF
            move_group_->setEndEffectorLink("link_tcp");
            
            RCLCPP_INFO(this->get_logger(), "MoveIt initialized for planning");
            return true;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveIt: %s", e.what());
            return false;
        }
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() >= 6) {
            // Store joint positions
            for (size_t i = 0; i < 6; i++) {
                current_joint_positions_[i] = msg->position[i];
            }
            
            // Update robot state and compute FK
            const moveit::core::JointModelGroup* joint_group = robot_model_->getJointModelGroup("xarm6_experiment");
            if (joint_group) {
                robot_state_->setJointGroupPositions(joint_group, current_joint_positions_);
                
                // Get TCP pose using FK
                const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform("link6");
                
                current_tcp_pose_.position.x = end_effector_state.translation().x();
                current_tcp_pose_.position.y = end_effector_state.translation().y();
                current_tcp_pose_.position.z = end_effector_state.translation().z();
                
                Eigen::Quaterniond q(end_effector_state.rotation());
                current_tcp_pose_.orientation.x = q.x();
                current_tcp_pose_.orientation.y = q.y();
                current_tcp_pose_.orientation.z = q.z();
                current_tcp_pose_.orientation.w = q.w();
            }
            
            if (!robot_state_received_) {
                robot_state_received_ = true;
                RCLCPP_INFO(this->get_logger(), "Robot state initialized - TCP at [%.3f, %.3f, %.3f]",
                           current_tcp_pose_.position.x,
                           current_tcp_pose_.position.y,
                           current_tcp_pose_.position.z);
            }
        }
    }
    
    geometry_msgs::msg::Pose getCurrentTcpPose()
    {
        return current_tcp_pose_;
    }
    
    bool planAndExecute(const geometry_msgs::msg::Pose& target_pose)
    {
        try {
            // Set robot state for planning (bypass state monitor)
            const moveit::core::JointModelGroup* joint_group = robot_model_->getJointModelGroup("xarm6_experiment");
            robot_state_->setJointGroupPositions(joint_group, current_joint_positions_);
            move_group_->setStartState(*robot_state_);
            
            // Set target
            move_group_->setPoseTarget(target_pose);
            
            // Plan
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
                // Execute
                success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            }
            
            move_group_->clearPoseTargets();
            return success;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Planning/execution error: %s", e.what());
            return false;
        }
    }
    
    geometry_msgs::msg::Pose mapControllerToTcp(const std::vector<double>& controller_pos_change, 
                                                double controller_y_rotation)
    {
        geometry_msgs::msg::Pose current_pose = getCurrentTcpPose();
        
        // Same smooth mapping as before
        std::vector<double> raw_pos_change(3);
        for (int i = 0; i < 3; i++) {
            raw_pos_change[i] = controller_pos_change[i] * POSITION_SCALE;
        }
        
        std::vector<double> tcp_pos_change(3);
        for (int i = 0; i < 3; i++) {
            tcp_pos_change[i] = std::tanh(raw_pos_change[i] / MAX_POSITION_CHANGE_PER_FRAME) * MAX_POSITION_CHANGE_PER_FRAME;
        }
        
        if (!smoothing_initialized_) {
            smoothed_position_ = tcp_pos_change;
            smoothing_initialized_ = true;
        } else {
            for (int i = 0; i < 3; i++) {
                smoothed_position_[i] = EMA_ALPHA * tcp_pos_change[i] + (1.0 - EMA_ALPHA) * smoothed_position_[i];
            }
        }
        tcp_pos_change = smoothed_position_;
        
        double mapped_rotation_change = std::tanh(controller_y_rotation / MAX_ROTATION_PER_FRAME) * MAX_ROTATION_PER_FRAME;
        smoothed_rotation_ = EMA_ALPHA * mapped_rotation_change + (1.0 - EMA_ALPHA) * smoothed_rotation_;
        
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.x += tcp_pos_change[0];
        target_pose.position.y += tcp_pos_change[1];  
        target_pose.position.z += tcp_pos_change[2];
        
        // Apply rotation change
        tf2::Quaternion current_q(current_pose.orientation.x,
                                 current_pose.orientation.y,
                                 current_pose.orientation.z,
                                 current_pose.orientation.w);
        
        tf2::Matrix3x3 current_m(current_q);
        double roll, pitch, yaw;
        current_m.getRPY(roll, pitch, yaw);
        yaw += smoothed_rotation_;
        
        tf2::Quaternion target_q;
        target_q.setRPY(roll, pitch, yaw);
        target_q.normalize();
        
        target_pose.orientation.x = target_q.x();
        target_pose.orientation.y = target_q.y();
        target_pose.orientation.z = target_q.z();
        target_pose.orientation.w = target_q.w();
        
        return target_pose;
    }
};

// Same mock VR interface
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
        double time_sec = clock_->now().seconds();
        std::vector<double> current_pos = {0.005 * std::sin(time_sec), 
                                          0.005 * std::cos(time_sec), 
                                          0.002 * std::sin(time_sec * 0.5)};
        double current_rot = 0.005 * std::sin(time_sec * 2.0);
        
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
    
    bool shouldExit() { return false; }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<DirectVRTeleop>();
        
        if (!node->initialize()) {
            RCLCPP_ERROR(node->get_logger(), "Failed to initialize");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Starting VR teleop loop...");
        MockVRInterface vr_interface(node);
        
        rclcpp::Rate loop_rate(5); // Slower rate: 5 Hz
        int loop_count = 0;
        
        while (rclcpp::ok()) {
            auto [pos_change, rot_change] = vr_interface.getControllerChange();
            
            if (loop_count < 10) {
                RCLCPP_INFO(node->get_logger(), "Warming up... %d/10", loop_count);
                rclcpp::spin_some(node);
                loop_count++;
                loop_rate.sleep();
                continue;
            }
            
            geometry_msgs::msg::Pose target_pose = node->mapControllerToTcp(pos_change, rot_change);
            
            bool success = node->planAndExecute(target_pose);
            
            if (loop_count % 10 == 0) {
                if (success) {
                    RCLCPP_INFO(node->get_logger(), "Motion successful at loop %d", loop_count);
                } else {
                    RCLCPP_WARN(node->get_logger(), "Motion failed at loop %d", loop_count);
                }
            }
            
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