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

class TFVRTeleop
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    
    // TF for getting current TCP pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Control parameters
    static constexpr double POSITION_SCALE = 0.8;
    static constexpr double MAX_POSITION_CHANGE = 0.01;
    static constexpr double MAX_ROTATION_CHANGE = 0.1;

public:
    TFVRTeleop(rclcpp::Node::SharedPtr node) : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "=== TF-based VR Teleop ===");
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create move group interface (for planning/execution only)
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "xarm6_experiment");
            
        move_group_->setPlanningTime(1.0);
        move_group_->setNumPlanningAttempts(3);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        
        // SET THE CORRECT END EFFECTOR LINK TO MATCH TF
        move_group_->setEndEffectorLink("link_tcp");
        
        RCLCPP_INFO(node_->get_logger(), "MoveIt initialized for planning/execution");
        RCLCPP_INFO(node_->get_logger(), "End effector set to: %s", move_group_->getEndEffectorLink().c_str());
        
        // Wait for TF to be available
        RCLCPP_INFO(node_->get_logger(), "Waiting for TF...");
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    geometry_msgs::msg::Pose getCurrentTcpPoseFromTF()
    {
        geometry_msgs::msg::Pose pose;
        
        try {
            // Get transform from world to link_tcp (TCP)
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "world", "link_tcp", tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            // Convert transform to pose
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            
            pose.orientation = transform.transform.rotation;
            
            return pose;
            
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(node_->get_logger(), "TF lookup failed: %s", ex.what());
            
            // Try alternative frame names
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "base_link", "link_tcp", tf2::TimePointZero, tf2::durationFromSec(1.0));
                
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                pose.orientation = transform.transform.rotation;
                
                return pose;
                
            } catch (const tf2::TransformException& ex2) {
                // Fallback to link6
                try {
                    geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                        "world", "link6", tf2::TimePointZero, tf2::durationFromSec(1.0));
                    
                    pose.position.x = transform.transform.translation.x;
                    pose.position.y = transform.transform.translation.y;
                    pose.position.z = transform.transform.translation.z;
                    pose.orientation = transform.transform.rotation;
                    
                    RCLCPP_WARN(node_->get_logger(), "Using link6 as fallback (link_tcp not found)");
                    return pose;
                    
                } catch (const tf2::TransformException& ex3) {
                    RCLCPP_ERROR(node_->get_logger(), "TF lookup failed for link_tcp and link6: %s", ex3.what());
                    
                    // Return zero pose as fallback
                    pose.position.x = pose.position.y = pose.position.z = 0.0;
                    pose.orientation.w = 1.0;
                    pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
                    return pose;
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
    
    geometry_msgs::msg::Pose applyControllerChanges(
        const geometry_msgs::msg::Pose& current_pose,
        const std::vector<double>& pos_change,
        double rotation_change)
    {
        geometry_msgs::msg::Pose target_pose = current_pose;
        
        // Apply position changes (scaled and limited)
        for (int i = 0; i < 3; i++) {
            double scaled_change = pos_change[i] * POSITION_SCALE;
            double limited_change = std::tanh(scaled_change / MAX_POSITION_CHANGE) * MAX_POSITION_CHANGE;
            
            if (i == 0) target_pose.position.x += limited_change;
            else if (i == 1) target_pose.position.y += limited_change;
            else if (i == 2) target_pose.position.z += limited_change;
        }
        
        // Apply rotation change to yaw
        tf2::Quaternion current_q(
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w);
            
        tf2::Matrix3x3 m(current_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        // Add limited rotation change
        double limited_rot = std::tanh(rotation_change / MAX_ROTATION_CHANGE) * MAX_ROTATION_CHANGE;
        yaw += limited_rot;
        
        tf2::Quaternion target_q;
        target_q.setRPY(roll, pitch, yaw);
        target_q.normalize();
        
        target_pose.orientation.x = target_q.x();
        target_pose.orientation.y = target_q.y();
        target_pose.orientation.z = target_q.z();
        target_pose.orientation.w = target_q.w();
        
        return target_pose;
    }
    
    void listAvailableFrames()
    {
        RCLCPP_INFO(node_->get_logger(), "Available TF frames:");
        auto frame_string = tf_buffer_->allFramesAsString();
        RCLCPP_INFO(node_->get_logger(), "%s", frame_string.c_str());
    }
};

// Mock VR interface
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
        // Very small test movements
        double time_sec = clock_->now().seconds();
        std::vector<double> current_pos = {
            0.001 * std::sin(time_sec * 0.5), 
            0.001 * std::cos(time_sec * 0.5), 
            0.0005 * std::sin(time_sec * 0.25)
        };
        double current_rot = 0.001 * std::sin(time_sec * 1.0);
        
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
    
    auto node = rclcpp::Node::make_shared("tf_vr_teleop");
    
    try {
        TFVRTeleop teleop(node);
        MockVRInterface vr_interface(node);
        
        // Show available frames
        teleop.listAvailableFrames();
        
        RCLCPP_INFO(node->get_logger(), "Getting initial TCP pose from TF...");
        
        // Get initial pose from TF
        auto current_pose = teleop.getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node->get_logger(), "Initial TCP from TF: [%.3f, %.3f, %.3f]",
                   current_pose.position.x, current_pose.position.y, current_pose.position.z);
        
        // Check if we got a valid pose
        if (current_pose.position.x == 0.0 && current_pose.position.y == 0.0 && current_pose.position.z == 0.0) {
            RCLCPP_WARN(node->get_logger(), "Got zero pose - TF might not be available");
            return 1;
        }
        
        RCLCPP_INFO(node->get_logger(), "Starting teleop loop...");
        
        rclcpp::Rate loop_rate(2); // Very slow: 2 Hz
        int loop_count = 0;
        
        while (rclcpp::ok() && loop_count < 10) { // Limited test
            auto [pos_change, rot_change] = vr_interface.getControllerChange();
            
            // Get current pose from TF each time
            current_pose = teleop.getCurrentTcpPoseFromTF();
            
            // Apply VR changes
            auto target_pose = teleop.applyControllerChanges(current_pose, pos_change, rot_change);
            
            // Show what we're trying to do
            RCLCPP_INFO(node->get_logger(), "[%d] TF pose: [%.3f,%.3f,%.3f] -> target: [%.3f,%.3f,%.3f]",
                       loop_count,
                       current_pose.position.x, current_pose.position.y, current_pose.position.z,
                       target_pose.position.x, target_pose.position.y, target_pose.position.z);
            
            // Execute motion
            bool success = teleop.moveToPose(target_pose);
            
            if (success) {
                RCLCPP_INFO(node->get_logger(), "[%d] Motion successful!", loop_count);
            } else {
                RCLCPP_WARN(node->get_logger(), "[%d] Motion failed", loop_count);
            }
            
            rclcpp::spin_some(node);
            loop_count++;
            loop_rate.sleep();
        }
        
        RCLCPP_INFO(node->get_logger(), "Test complete!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}