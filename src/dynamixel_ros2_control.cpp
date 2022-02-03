// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "dynamixel_ros2_control/dynamixel_ros2_controll.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dynamixel_ros2_control/dynamixel_driver.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cctype>

#define DYN_LOGGER_NAME_STR "Dynamixel_Hardware_Interface"

#define DYN_BAUD_PARAM_STR "baud_rate"
#define DYN_PORTNAME_PARAM_STR "serial_port"
#define DYN_ENABLE_TORQUE_ON_START_PARAM_STR "enable_torque_on_startup"
#define DYN_AUTO_REBOOT_PARAM_STR "reboot_on_error"
#define DYN_ID_PARAM_STR "id"

#define DYN_DEFAULT_PORT_STR "/dev/ttyUSB0"

namespace dynamixel_ros2_control {

CallbackReturn DynamixelHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR), "configure");
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  auto serial_port_it = info_.hardware_parameters.find(DYN_PORTNAME_PARAM_STR);
  std::string serial_port = (serial_port_it != info_.hardware_parameters.end()
                                 ? serial_port_it->second
                                 : std::string(DYN_DEFAULT_PORT_STR));
  auto baud_rate_it = info_.hardware_parameters.find(DYN_BAUD_PARAM_STR);
  int baud_rate = std::stoi(baud_rate_it != info_.hardware_parameters.end()
                                ? baud_rate_it->second
                                : "57600");
  dynamixel_driver_ =
      std::make_unique<dynamixel::Driver>(serial_port, baud_rate);
  RCLCPP_INFO(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
              ("Serial Port \"" + serial_port +
               "\" (baud rate: " + std::to_string(baud_rate) + ")")
                  .c_str());

  dynamixel_driver_->set_hardware_error_callbacks(
      std::bind(&DynamixelHardwareInterface::overload_error_callback, this,
                std::placeholders::_1),
      std::bind(&DynamixelHardwareInterface::input_voltage_error_callback, this,
                std::placeholders::_1),
      std::bind(&DynamixelHardwareInterface::overheating_error_callback, this,
                std::placeholders::_1),
      std::bind(&DynamixelHardwareInterface::electrical_shock_error_callback,
                this, std::placeholders::_1),
      std::bind(&DynamixelHardwareInterface::motor_encoder_error_callback, this,
                std::placeholders::_1));

  std::map<std::string, dynamixel::motor_id> motorIds;
  for (const auto &joint : info_.joints) {
    motorIds[joint.name] = std::stoi(joint.parameters.at(DYN_ID_PARAM_STR));
    auto auto_reboot = joint.parameters.find(DYN_AUTO_REBOOT_PARAM_STR);
    auto_clear_overload_error[joint.name] =
        (auto_reboot != joint.parameters.end())
            ? (auto_reboot->second == "true")
            : false;
  }

  bool failure = false;
  for (auto const &x : motorIds) {
    try {
      dynamixel_driver_->add_motor(x.first, x.second);
      RCLCPP_INFO(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
                  ("Joint-Actuator \"" + std::string(x.first) +
                   "\" (ID: " + std::to_string(x.second) + ") added")
                      .c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
                   ("Joint-Actuator \"" + std::string(x.first) +
                    "\"(ID: " + std::to_string(x.second) +
                    ") could not be added! \n" + e.what())
                       .c_str());
      failure = true;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
              (std::string("Motors count: ") +
               std::to_string(dynamixel_driver_->get_motor_count()))
                  .c_str());
  if (failure) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

void DynamixelHardwareInterface::input_voltage_error_callback(
    const std::string &joint_name) const {
  RCLCPP_ERROR(
      rclcpp::get_logger(DYN_LOGGER_NAME_STR),
      ("input voltage of joint \"" + joint_name + "\" not correct!").c_str());
}
void DynamixelHardwareInterface::overheating_error_callback(
    const std::string &joint_name) const {
  RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
               ("Joint \"" + joint_name + "\" is overheating!").c_str());
}
void DynamixelHardwareInterface::motor_encoder_error_callback(
    const std::string &joint_name) const {
  RCLCPP_ERROR(
      rclcpp::get_logger(DYN_LOGGER_NAME_STR),
      ("Motor encoder error on joint \"" + joint_name + "\"!").c_str());
}
void DynamixelHardwareInterface::electrical_shock_error_callback(
    const std::string &joint_name) const {
  RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
               ("Electrical shock on joint \"" + joint_name + "\": !").c_str());
}
void DynamixelHardwareInterface::overload_error_callback(
    const std::string &joint_name) const {
  RCLCPP_ERROR(
      rclcpp::get_logger(DYN_LOGGER_NAME_STR),
      ("Mechanical overload error on joint \"" + joint_name + "\": !").c_str());
  if (auto_clear_overload_error.at(joint_name)) {
    try {
      dynamixel_driver_->reboot(joint_name);
      RCLCPP_INFO(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
                  ("\"" + joint_name + " rebootet").c_str());
      // succeeded = true;
    } catch (std::invalid_argument &e) {
      RCLCPP_WARN(rclcpp::get_logger(DYN_LOGGER_NAME_STR), "%s", e.what());
    }
  }
}

std::vector<hardware_interface::StateInterface>
DynamixelHardwareInterface::export_state_interfaces() {
  RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
               "export_state_interfaces");
  std::vector<hardware_interface::StateInterface> state_interfaces;
  dynamixel_driver_->for_each_joint([&](const std::string &joint_name) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_POSITION,
        dynamixel_driver_->get_position_ptr(joint_name)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_VELOCITY,
        dynamixel_driver_->get_velocity_ptr(joint_name)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_name, hardware_interface::HW_IF_EFFORT,
        dynamixel_driver_->get_effort_ptr(joint_name)));
  });
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelHardwareInterface::export_command_interfaces() {
  RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
               "export_command_interfaces");
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  dynamixel_driver_->for_each_joint([&](const std::string &joint_name) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_name, hardware_interface::HW_IF_POSITION,
        dynamixel_driver_->get_goal_position_ptr(joint_name)));
  });
  return command_interfaces;
}

CallbackReturn DynamixelHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR), "on_activate");
  try {
    dynamixel_driver_->ping_all();
    RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR),
                 "Pinging dynamixels was successfull");
  } catch (const dynamixel::Driver::dynamixel_bus_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR), e.what());
    return CallbackReturn::ERROR;
  }

  try {
    dynamixel_driver_->read();
    dynamixel_driver_->set_torque_all(true);
    dynamixel_driver_->write();
  } catch (std::invalid_argument &e) {
    RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR), e.what());
    return CallbackReturn::ERROR;
  } catch (const dynamixel::Driver::hardware_error &e) {
    RCLCPP_ERROR(rclcpp::get_logger(DYN_LOGGER_NAME_STR), e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DynamixelHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_DEBUG(rclcpp::get_logger(DYN_LOGGER_NAME_STR), "on_deactivate");
  try {
    dynamixel_driver_->read();
    dynamixel_driver_->set_torque_all(false);
    dynamixel_driver_->write();
    RCLCPP_INFO(rclcpp::get_logger(DYN_LOGGER_NAME_STR), "Motors deaktivated. Torque disabled !");
  } catch (std::invalid_argument &e) {
    RCLCPP_WARN(rclcpp::get_logger(DYN_LOGGER_NAME_STR), e.what());
    return CallbackReturn::ERROR;
  } catch (const dynamixel::Driver::hardware_error &e) {
    RCLCPP_WARN(rclcpp::get_logger(DYN_LOGGER_NAME_STR), e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardwareInterface::read() {
  dynamixel_driver_->read();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DynamixelHardwareInterface::write() {
  dynamixel_driver_->write(true);
  return hardware_interface::return_type::OK;
}

} // namespace dynamixel_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(dynamixel_ros2_control::DynamixelHardwareInterface,
                       hardware_interface::SystemInterface)