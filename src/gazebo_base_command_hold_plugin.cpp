#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo_ros/node.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>
#include <ignition/math/Pose3.hh>
#include <rclcpp/rclcpp.hpp>

namespace gazebo
{
class BaseCommandHoldPlugin : public ModelPlugin
{
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    if (!ros_node_) {
      gzerr << "BaseCommandHoldPlugin failed to create a ROS node\n";
      return;
    }

    command_topic_ = "/windylab/base_command";
    if (sdf->HasElement("command_topic")) {
      command_topic_ = sdf->Get<std::string>("command_topic");
    }
    if (sdf->HasElement("hold_every_n_updates")) {
      hold_every_n_updates_ = sdf->Get<std::uint64_t>("hold_every_n_updates");
    }
    if (hold_every_n_updates_ == 0U) {
      gzerr << "BaseCommandHoldPlugin hold_every_n_updates must be positive\n";
      return;
    }

    command_subscription_ = ros_node_->create_subscription<gazebo_msgs::msg::EntityState>(
      command_topic_, rclcpp::QoS(1).reliable(),
      [this](gazebo_msgs::msg::EntityState::SharedPtr message) {
        this->OnCommand(*message);
      });
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BaseCommandHoldPlugin::OnUpdate, this, std::placeholders::_1));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Holding model '%s' from %s every %llu Gazebo physics updates",
      model_->GetName().c_str(), command_topic_.c_str(),
      static_cast<unsigned long long>(hold_every_n_updates_));
  }

private:
  struct Command
  {
    ignition::math::Pose3d pose;
    ignition::math::Vector3d linear_velocity;
    ignition::math::Vector3d angular_velocity;
    std::uint64_t generation{0U};
    bool valid{false};
  };

  static bool IsFinite(const gazebo_msgs::msg::EntityState & message)
  {
    const auto & position = message.pose.position;
    const auto & orientation = message.pose.orientation;
    const auto & linear = message.twist.linear;
    const auto & angular = message.twist.angular;
    return
      std::isfinite(position.x) && std::isfinite(position.y) &&
      std::isfinite(position.z) && std::isfinite(orientation.x) &&
      std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
      std::isfinite(orientation.w) && std::isfinite(linear.x) &&
      std::isfinite(linear.y) && std::isfinite(linear.z) &&
      std::isfinite(angular.x) && std::isfinite(angular.y) &&
      std::isfinite(angular.z);
  }

  void OnCommand(const gazebo_msgs::msg::EntityState & message)
  {
    if (!model_ || (!message.name.empty() && message.name != model_->GetName())) {
      return;
    }
    if (!message.reference_frame.empty() && message.reference_frame != "world") {
      RCLCPP_WARN_ONCE(
        ros_node_->get_logger(),
        "Ignoring Base commands outside the world reference frame");
      return;
    }
    if (!IsFinite(message)) {
      RCLCPP_WARN_ONCE(ros_node_->get_logger(), "Ignoring a non-finite Base command");
      return;
    }

    const auto & q = message.pose.orientation;
    const double quaternion_norm = std::sqrt(
      q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (quaternion_norm < 1e-9) {
      RCLCPP_WARN_ONCE(ros_node_->get_logger(), "Ignoring a zero-norm Base quaternion");
      return;
    }

    Command command;
    command.pose = ignition::math::Pose3d(
      ignition::math::Vector3d(
        message.pose.position.x,
        message.pose.position.y,
        message.pose.position.z),
      ignition::math::Quaterniond(
        q.w / quaternion_norm,
        q.x / quaternion_norm,
        q.y / quaternion_norm,
        q.z / quaternion_norm));
    command.linear_velocity.Set(
      message.twist.linear.x,
      message.twist.linear.y,
      message.twist.linear.z);
    command.angular_velocity.Set(
      message.twist.angular.x,
      message.twist.angular.y,
      message.twist.angular.z);
    command.valid = true;

    std::lock_guard<std::mutex> lock(command_mutex_);
    command.generation = ++command_generation_;
    command_ = command;
  }

  void OnUpdate(const common::UpdateInfo & update_info)
  {
    Command command;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      command = command_;
    }
    if (!command.valid || !model_) {
      return;
    }
    if (command.generation != applied_generation_) {
      applied_generation_ = command.generation;
      command_start_sim_time_ = update_info.simTime;
    }
    ++update_count_;
    if (update_count_ % hold_every_n_updates_ != 0U) {
      return;
    }

    const double command_age_sec =
      (update_info.simTime - command_start_sim_time_).Double();
    ignition::math::Pose3d extrapolated_pose = command.pose;
    extrapolated_pose.Pos() += command.linear_velocity * command_age_sec;
    model_->SetWorldPose(extrapolated_pose, true, false);
    model_->SetLinearVel(command.linear_velocity);
    model_->SetAngularVel(command.angular_velocity);
  }

  physics::ModelPtr model_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<gazebo_msgs::msg::EntityState>::SharedPtr command_subscription_;
  event::ConnectionPtr update_connection_;
  std::string command_topic_;
  std::mutex command_mutex_;
  Command command_;
  common::Time command_start_sim_time_;
  std::uint64_t command_generation_{0U};
  std::uint64_t applied_generation_{0U};
  std::uint64_t update_count_{0U};
  std::uint64_t hold_every_n_updates_{6U};
};

GZ_REGISTER_MODEL_PLUGIN(BaseCommandHoldPlugin)
}  // namespace gazebo
