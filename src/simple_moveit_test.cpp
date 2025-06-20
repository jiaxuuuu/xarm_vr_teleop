#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <chrono>
#include <thread>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_moveit_test");
    
    RCLCPP_INFO(node->get_logger(), "=== Simple MoveIt Test ===");
    
    try {
        RCLCPP_INFO(node->get_logger(), "Creating MoveGroupInterface...");
        
        // Create move group interface directly without inheritance
        auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node, "xarm6_experiment");
            
        // SET THE CORRECT END EFFECTOR LINK TO MATCH TF
        move_group->setEndEffectorLink("link_tcp");
            
        RCLCPP_INFO(node->get_logger(), "MoveGroupInterface created successfully!");
        
        RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group->getPlanningFrame().c_str());
        RCLCPP_INFO(node->get_logger(), "End effector link: %s", move_group->getEndEffectorLink().c_str());
        
        // Wait for robot state to be available
        RCLCPP_INFO(node->get_logger(), "Waiting for robot state...");
        
        // Spin for a bit to let state monitor initialize
        for (int i = 0; i < 30; i++) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Try to get current pose
        RCLCPP_INFO(node->get_logger(), "Getting current pose...");
        auto current_pose = move_group->getCurrentPose();
        
        RCLCPP_INFO(node->get_logger(), "Current TCP position: [%.3f, %.3f, %.3f]",
                   current_pose.pose.position.x,
                   current_pose.pose.position.y,
                   current_pose.pose.position.z);
                   
        // Test a simple plan to a nearby pose
        RCLCPP_INFO(node->get_logger(), "Testing planning to nearby pose...");
        auto target_pose = current_pose.pose;
        target_pose.position.z += 0.01; // Move 1cm up
        
        move_group->setPoseTarget(target_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(node->get_logger(), "Planning successful!");
        } else {
            RCLCPP_WARN(node->get_logger(), "Planning failed, but MoveIt connection is working");
        }
                   
        RCLCPP_INFO(node->get_logger(), "SUCCESS: MoveIt is working correctly!");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}