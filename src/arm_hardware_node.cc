#include <arm_platform/arm_hardware_node.h>
#include <arm_platform/bus/serial_bus.h>
#include <arm_platform/protocol/dummy_link_protocol.h>
#include <arm_platform/arm_model/dummy_arm.h>
#include <arm_platform/controller/dummy_controller.h>

using namespace std::chrono_literals;

namespace arm_platform {

ArmHardwareNode::ArmHardwareNode() : Node("robot_arm_node") {

    // ------------------- Hardware stack -------------------
    std::string port;
    this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
    this->get_parameter("port_name", port);

    auto bus = std::make_shared<arm_platform::bus::SerialBus>(port, 921600);
    auto protocol = std::make_shared<arm_platform::protocol::DummyLinkProtocol>();
    auto arm_model = std::make_shared<arm_platform::arm_model::DummyArm>();

    num_joints_ = static_cast<uint8_t>(arm_model->Dof());

    controller_ = std::make_shared<arm_platform::controller::DummyController>(
        arm_model, protocol, bus);

    // ------------------- ROS interfaces -------------------
    pub_joint_state_ = this->create_publisher<dummy_interface::msg::MotorState>(
        "arm/joint_feedback", 10);

    sub_joint_ctrl_ = this->create_subscription<dummy_interface::msg::MotorControl>(
        "arm/joint_control", 10,
        std::bind(&ArmHardwareNode::JointControlCallback, this, std::placeholders::_1));

    InitCommand();

    // ------------------- Control loop timer -------------------
    control_timer_ = this->create_wall_timer(
        5ms,   // 200Hz
        std::bind(&ArmHardwareNode::ControlLoop, this));

    RCLCPP_INFO(this->get_logger(), "ArmHardwareNode initialized (200Hz control loop)");
}

ArmHardwareNode::~ArmHardwareNode() = default;

// --------------------------------------------------------
// Initialize default command (MIT-like gains)
// --------------------------------------------------------
void ArmHardwareNode::InitCommand() {
    arm_target_cmd_.position.resize(num_joints_, 0.0);
    arm_target_cmd_.velocity.resize(num_joints_, 0.0);
    arm_target_cmd_.current.resize(num_joints_, 0.0);
    arm_target_cmd_.p.resize(num_joints_, 2.5);
    arm_target_cmd_.d.resize(num_joints_, 0.8);

    // Example safety limits
    if (num_joints_ >= 7) {
        arm_target_cmd_.velocity[3] = 10.0;
        arm_target_cmd_.velocity[4] = 10.0;
        arm_target_cmd_.velocity[5] = 10.0;
        arm_target_cmd_.velocity[6] = 100.0;

        arm_target_cmd_.current[3] = 2000.0;
        arm_target_cmd_.current[4] = 2000.0;
        arm_target_cmd_.current[5] = 2000.0;
        arm_target_cmd_.current[6] = 2000.0;
    }
}

// --------------------------------------------------------
// 200Hz real-time control loop
// --------------------------------------------------------
void ArmHardwareNode::ControlLoop() {
    // 1) Copy target command (short lock)
    JointCommand cmd;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd = arm_target_cmd_;
    }

    // 2) Send command to controller
    controller_->SetCommand(cmd);

    controller_->Update(arm_current_state_);

    // 5) Publish to ROS
    dummy_interface::msg::MotorState msg;
    msg.header.stamp = this->now();

    for (size_t i = 0; i < num_joints_; ++i) {
        msg.position.push_back(arm_current_state_.position[i]);
        msg.velocity.push_back(arm_current_state_.velocity[i]);
        msg.current.push_back(arm_current_state_.current[i]);
        msg.voltage.push_back(arm_current_state_.voltage[i]);
        msg.temperature.push_back(arm_current_state_.temperature[i]);
    }

    pub_joint_state_->publish(msg);
}

// --------------------------------------------------------
// Incoming ROS command → target command
// --------------------------------------------------------
void ArmHardwareNode::JointControlCallback(
    const dummy_interface::msg::MotorControl::SharedPtr msg) {

    std::lock_guard<std::mutex> lock(mutex_);

    for (size_t i = 0; i < std::min<size_t>(msg->position.size(), num_joints_); ++i)
        arm_target_cmd_.position[i] = msg->position[i];

    for (size_t i = 0; i < std::min<size_t>(msg->velocity.size(), num_joints_); ++i)
        arm_target_cmd_.velocity[i] = msg->velocity[i];

    for (size_t i = 0; i < std::min<size_t>(msg->current.size(), num_joints_); ++i)
        arm_target_cmd_.current[i] = msg->current[i];

    for (size_t i = 0; i < std::min<size_t>(msg->p.size(), num_joints_); ++i)
        arm_target_cmd_.p[i] = msg->p[i];

    for (size_t i = 0; i < std::min<size_t>(msg->d.size(), num_joints_); ++i)
        arm_target_cmd_.d[i] = msg->d[i];
}

} // namespace arm_platform

// --------------------------------------------------------
// main()
// --------------------------------------------------------
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<arm_platform::ArmHardwareNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
