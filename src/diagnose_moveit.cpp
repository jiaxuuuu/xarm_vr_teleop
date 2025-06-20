#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <chrono>
#include <thread>

class MoveItDiagnostic : public rclcpp::Node
{
public:
    MoveItDiagnostic() : Node("moveit_diagnostic")
    {
        RCLCPP_INFO(this->get_logger(), "=== MoveIt Diagnostic Tool ===");
        
        // Check available topics
        checkTopics();
        
        // Try to load robot model
        checkRobotModel();
        
        // Subscribe to joint states
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&MoveItDiagnostic::jointStateCallback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
    }
    
private:
    void checkTopics()
    {
        RCLCPP_INFO(this->get_logger(), "\n--- Checking Topics ---");
        auto topic_names = this->get_topic_names_and_types();
        
        bool found_joint_states = false;
        for (const auto& [name, types] : topic_names) {
            if (name.find("joint") != std::string::npos || 
                name.find("state") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Found topic: %s", name.c_str());
                if (name == "/joint_states") {
                    found_joint_states = true;
                }
            }
        }
        
        if (!found_joint_states) {
            RCLCPP_WARN(this->get_logger(), "No /joint_states topic found!");
        }
    }
    
    void checkRobotModel()
    {
        RCLCPP_INFO(this->get_logger(), "\n--- Checking Robot Model ---");
        try {
            robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
            moveit::core::RobotModelPtr robot_model = robot_model_loader.getModel();
            
            if (!robot_model) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load robot model");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Robot model loaded successfully");
            RCLCPP_INFO(this->get_logger(), "Model name: %s", robot_model->getName().c_str());
            
            // List joint model groups
            const std::vector<std::string>& group_names = robot_model->getJointModelGroupNames();
            RCLCPP_INFO(this->get_logger(), "Available planning groups:");
            for (const auto& group : group_names) {
                RCLCPP_INFO(this->get_logger(), "  - %s", group.c_str());
            }
            
            // Check specific group
            const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup("xarm6_experiment");
            if (joint_model_group) {
                RCLCPP_INFO(this->get_logger(), "Found xarm6_experiment group");
                const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
                RCLCPP_INFO(this->get_logger(), "Joints in group:");
                for (const auto& joint : joint_names) {
                    RCLCPP_INFO(this->get_logger(), "  - %s", joint.c_str());
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "xarm6_experiment group not found!");
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Robot model error: %s", e.what());
        }
    }
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        static int count = 0;
        if (count == 0) {
            RCLCPP_INFO(this->get_logger(), "\n--- Joint States Received ---");
            RCLCPP_INFO(this->get_logger(), "Number of joints: %zu", msg->name.size());
            RCLCPP_INFO(this->get_logger(), "Joint names:");
            for (size_t i = 0; i < msg->name.size(); i++) {
                RCLCPP_INFO(this->get_logger(), "  [%zu] %s = %.3f", 
                           i, msg->name[i].c_str(), 
                           i < msg->position.size() ? msg->position[i] : 0.0);
            }
        }
        count++;
        
        if (count % 50 == 0) {
            RCLCPP_INFO(this->get_logger(), "Joint states still coming... (count: %d)", count);
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MoveItDiagnostic>();
    
    RCLCPP_INFO(node->get_logger(), "Running diagnostic for 10 seconds...");
    
    // Run for 10 seconds
    rclcpp::spin_some(node);
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        auto now = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start);
        if (duration.count() >= 10) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    RCLCPP_INFO(node->get_logger(), "Diagnostic complete");
    rclcpp::shutdown();
    return 0;
}