#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>

class SimpleMovementTest
{
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    rclcpp::Node::SharedPtr node_;
    
    // TF for getting current TCP pose
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
    SimpleMovementTest(rclcpp::Node::SharedPtr node) : node_(node)
    {
        RCLCPP_INFO(node_->get_logger(), "=== Simple Movement Test ===");
        
        // Initialize TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Create move group interface
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_, "xarm6_experiment");
            
        move_group_->setPlanningTime(2.0);
        move_group_->setNumPlanningAttempts(3);
        move_group_->setMaxVelocityScalingFactor(0.1);
        move_group_->setMaxAccelerationScalingFactor(0.1);
        
        // Print MoveIt configuration BEFORE setting end effector
        RCLCPP_INFO(node_->get_logger(), "MoveIt Configuration (before):");
        RCLCPP_INFO(node_->get_logger(), "  Planning frame: %s", move_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(node_->get_logger(), "  End effector link: %s", move_group_->getEndEffectorLink().c_str());
        
        // SET THE CORRECT END EFFECTOR LINK
        move_group_->setEndEffectorLink("link_tcp");
        
        // Print MoveIt configuration AFTER setting end effector
        RCLCPP_INFO(node_->get_logger(), "MoveIt Configuration (after):");
        RCLCPP_INFO(node_->get_logger(), "  End effector link: %s", move_group_->getEndEffectorLink().c_str());
        
        // Wait for TF
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    
    geometry_msgs::msg::Pose getCurrentTcpPoseFromTF()
    {
        geometry_msgs::msg::Pose pose;
        
        try {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                "world", "link_tcp", tf2::TimePointZero, tf2::durationFromSec(1.0));
            
            pose.position.x = transform.transform.translation.x;
            pose.position.y = transform.transform.translation.y;
            pose.position.z = transform.transform.translation.z;
            pose.orientation = transform.transform.rotation;
            
            return pose;
            
        } catch (const tf2::TransformException& ex) {
            try {
                geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
                    "world", "link6", tf2::TimePointZero, tf2::durationFromSec(1.0));
                
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                pose.orientation = transform.transform.rotation;
                
                RCLCPP_WARN_ONCE(node_->get_logger(), "Using link6 fallback");
                return pose;
                
            } catch (const tf2::TransformException& ex2) {
                RCLCPP_ERROR(node_->get_logger(), "TF failed: %s", ex2.what());
                pose.position.x = pose.position.y = pose.position.z = 0.0;
                pose.orientation.w = 1.0;
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
    
    void testSimpleMovements()
    {
        RCLCPP_INFO(node_->get_logger(), "Testing SIMPLE movements to isolate mapping issues");
        
        auto initial_pose = getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node_->get_logger(), "Initial TF pose: [%.3f, %.3f, %.3f]",
                   initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        
        // Compare with MoveIt's current pose
        auto moveit_pose = move_group_->getCurrentPose();
        RCLCPP_INFO(node_->get_logger(), "MoveIt pose:    [%.3f, %.3f, %.3f]",
                   moveit_pose.pose.position.x, moveit_pose.pose.position.y, moveit_pose.pose.position.z);
        
        // Check if they match
        double diff_x = std::abs(initial_pose.position.x - moveit_pose.pose.position.x);
        double diff_y = std::abs(initial_pose.position.y - moveit_pose.pose.position.y);
        double diff_z = std::abs(initial_pose.position.z - moveit_pose.pose.position.z);
        
        if (diff_x > 0.01 || diff_y > 0.01 || diff_z > 0.01) {
            RCLCPP_ERROR(node_->get_logger(), "❌ TF and MoveIt poses don't match! Diff: [%.3f, %.3f, %.3f]",
                        diff_x, diff_y, diff_z);
        } else {
            RCLCPP_INFO(node_->get_logger(), "✅ TF and MoveIt poses match");
        }
        
        // Test 1: Move 1cm in +X direction
        RCLCPP_INFO(node_->get_logger(), "\n=== TEST 1: +X movement (1cm) ===");
        auto target1 = initial_pose;
        target1.position.x += 0.01;  // 1cm in +X
        
        RCLCPP_INFO(node_->get_logger(), "Target: [%.3f, %.3f, %.3f]",
                   target1.position.x, target1.position.y, target1.position.z);
        
        bool success1 = moveToPose(target1);
        RCLCPP_INFO(node_->get_logger(), "Result: %s", success1 ? "SUCCESS" : "FAILED");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        // Check actual position after movement
        auto actual1 = getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node_->get_logger(), "Actual: [%.3f, %.3f, %.3f]",
                   actual1.position.x, actual1.position.y, actual1.position.z);
        
        double error_x = actual1.position.x - target1.position.x;
        double error_y = actual1.position.y - target1.position.y;
        double error_z = actual1.position.z - target1.position.z;
        RCLCPP_INFO(node_->get_logger(), "Error: [%.3f, %.3f, %.3f]", error_x, error_y, error_z);
        
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Test 2: Move 1cm in +Y direction from current position
        RCLCPP_INFO(node_->get_logger(), "\n=== TEST 2: +Y movement (1cm) ===");
        auto current_pose = getCurrentTcpPoseFromTF();
        auto target2 = current_pose;
        target2.position.y += 0.01;  // 1cm in +Y
        
        RCLCPP_INFO(node_->get_logger(), "Target: [%.3f, %.3f, %.3f]",
                   target2.position.x, target2.position.y, target2.position.z);
        
        bool success2 = moveToPose(target2);
        RCLCPP_INFO(node_->get_logger(), "Result: %s", success2 ? "SUCCESS" : "FAILED");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        auto actual2 = getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node_->get_logger(), "Actual: [%.3f, %.3f, %.3f]",
                   actual2.position.x, actual2.position.y, actual2.position.z);
        
        // Test 3: Move 1cm in +Z direction from current position
        RCLCPP_INFO(node_->get_logger(), "\n=== TEST 3: +Z movement (1cm) ===");
        current_pose = getCurrentTcpPoseFromTF();
        auto target3 = current_pose;
        target3.position.z += 0.01;  // 1cm in +Z
        
        RCLCPP_INFO(node_->get_logger(), "Target: [%.3f, %.3f, %.3f]",
                   target3.position.x, target3.position.y, target3.position.z);
        
        bool success3 = moveToPose(target3);
        RCLCPP_INFO(node_->get_logger(), "Result: %s", success3 ? "SUCCESS" : "FAILED");
        
        std::this_thread::sleep_for(std::chrono::seconds(3));
        
        auto actual3 = getCurrentTcpPoseFromTF();
        RCLCPP_INFO(node_->get_logger(), "Actual: [%.3f, %.3f, %.3f]",
                   actual3.position.x, actual3.position.y, actual3.position.z);
        
        // Summary
        RCLCPP_INFO(node_->get_logger(), "\n=== SUMMARY ===");
        RCLCPP_INFO(node_->get_logger(), "Initial: [%.3f, %.3f, %.3f]",
                   initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        RCLCPP_INFO(node_->get_logger(), "Final:   [%.3f, %.3f, %.3f]",
                   actual3.position.x, actual3.position.y, actual3.position.z);
        RCLCPP_INFO(node_->get_logger(), "Total movement: [%.3f, %.3f, %.3f]",
                   actual3.position.x - initial_pose.position.x,
                   actual3.position.y - initial_pose.position.y,
                   actual3.position.z - initial_pose.position.z);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("simple_movement_test");
    
    try {
        SimpleMovementTest test(node);
        test.testSimpleMovements();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}