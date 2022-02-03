#pragma once
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>


#include "dynamixel_ros2_control/visiblity_control.h"
#include "dynamixel_ros2_control/dynamixel_driver.hpp"
#include "rclcpp/macros.hpp"
#include <map>
#include <memory>

namespace dynamixel_ros2_control 
{

class DynamixelHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DynamixelHardwareInterface)

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  DYNAMIXEL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  hardware_interface::return_type enable_torque(const bool enabled);

  void input_voltage_error_callback(const std::string &joint_name) const;
  void overheating_error_callback(const std::string &joint_name) const;
  void motor_encoder_error_callback(const std::string &joint_name) const;
  void electrical_shock_error_callback(const std::string &joint_name) const;
  void overload_error_callback(const std::string &joint_name) const;

  std::unique_ptr<dynamixel::Driver> dynamixel_driver_;
  std::map<std::string, bool> auto_clear_overload_error;
};

} // namespace dynamixel_ros2_control
