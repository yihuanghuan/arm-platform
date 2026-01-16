#include <arm_platform/moveit_bridge_node.h>

namespace arm_platform {

MoveItBridgeNode::MoveItBridgeNode() : Node("moveit_bridge_node") {
   
    // --- Publisher / Subscriber ---
    pub_joint_ctrl_ = this->create_publisher<dummy_interface::msg::MotorControl>("arm/joint_control", 10);

    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10, std::bind(&MoveItBridgeNode::JoinStateCallback, this, std::placeholders::_1));

    // --- Initialize joint name mapping ---
    ordered_joint_names_ = {"joint1","joint2","joint3","joint4","joint5","joint6","joint7"};
    RCLCPP_INFO(this->get_logger(), "MoveItBridgeNode initialized!");
}

MoveItBridgeNode::~MoveItBridgeNode() {
    
}

// --- Callback: trajectory from MoveIt ---
void MoveItBridgeNode::JoinStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {

    // Get joint mapping once
    if(joint_indices_.empty()) {
        for(auto &name : msg->name) {
            auto it = std::find(ordered_joint_names_.begin(), ordered_joint_names_.end(), name);
            if(it != ordered_joint_names_.end()) {
                joint_indices_.push_back(std::distance(ordered_joint_names_.begin(), it));
            }
        }
    }

    auto joint_cmd = dummy_interface::msg::MotorControl();
    joint_cmd.position.resize(7, 0.0);

    for(size_t i=0; i<std::min(msg->position.size(), joint_cmd.position.size()); ++i) {
        joint_cmd.position[joint_indices_[i]] = msg->position[i];
    }

    pub_joint_ctrl_->publish(joint_cmd);
}

} // namespace arm_platform::node

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_platform::MoveItBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
