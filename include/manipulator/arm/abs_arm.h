#pragma once
#include <vector>
#include <memory>
#include <map>
#include <string>

#include <sensor_msgs/msg/joint_state.hpp>
#include <dummy_interface/msg/motor_control.hpp>
#include <manipulator/arm/i_arm.h>
#include <manipulator/motor/i_motor.h>
#include <manipulator/bus/i_bus.h>

namespace manipulator::arm {

/**
 * @brief Abstract base class for robotic arm implementations
 * 
 * This class provides common functionality for all arm types
 * including motor management, bus communication, and joint state handling.
 * It implements the IArm interface and manages hardware communication.
 */
class AbsArm : public IArm {
 public:
  /**
   * @brief Default constructor
   */
  AbsArm();

  /**
   * @brief Constructor with bus initialization
   * @param bus Unique pointer to communication bus
   */
  explicit AbsArm(bus::IBus::UniquePtr bus);
    
  /**
   * @brief Delete copy constructor (UniquePtr is not copyable)
   */
  AbsArm(const AbsArm&) = delete;
  
  /**
   * @brief Delete copy assignment operator (UniquePtr is not copyable)
   */
  AbsArm& operator=(const AbsArm&) = delete;   
  
  /**
   * @brief Default destructor
   */
  ~AbsArm() = default;

  /**
   * @brief Set communication bus for the arm
   * @param bus Unique pointer to communication bus
   */
  void SetBus(bus::IBus::UniquePtr bus);

  /**
   * @brief Send motor command to hardware
   * @param cmd Motor control message containing current, position, velocity, etc.
   * @return True if command was sent successfully, false otherwise
   */
  bool SetMotorCommand(const dummy_interface::msg::MotorControl& cmd);

  /**
   * @brief Set joint states (for simulation or control)
   * @param state Joint state message
   * @return True if states were set successfully, false otherwise
   */
  bool SetJointStates(const sensor_msgs::msg::JointState& state);

  /**
   * @brief Get current joint states from hardware
   * 
   * This virtual method should be overridden by derived classes
   * to read joint states from actual hardware.
   */
  virtual void GetJointStates();

  /**
   * @brief Add a motor to the arm
   * @param name Motor name/identifier
   * @param motor Shared pointer to motor instance
   */
  void AddMotor(const std::string& name, motor::IMotor::SharedPtr motor);

  /**
   * @brief Remove a motor from the arm
   * @param name Motor name/identifier
   */
  void RemoveMotor(const std::string& name);

// protected:
//     virtual bool CheckJointLimits() const;
 private:
  /**
   * @brief Communication bus for hardware interaction
   */
  bus::IBus::UniquePtr bus_;
  
  /**
   * @brief Map of motor name to motor instance
   */
  std::map<std::string, motor::IMotor::SharedPtr> motor_map_;

  // std::vector<JointLimit> joint_limits_;
};

} // namespace manipulator::arm