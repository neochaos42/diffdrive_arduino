#include "diffdrive_arduino/fake_robot.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

FakeRobot::FakeRobot()
  : logger_(rclcpp::get_logger("FakeRobot"))
{}

return_type FakeRobot::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["bleft_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["bright_wheel_name"];

  // Set up the wheels
  fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> FakeRobot::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Export state interfaces for all four wheels
  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_POSITION, &bl_wheel_.pos));
  
  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeRobot::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Export command interfaces for all four wheels
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));
  
  command_interfaces.emplace_back(hardware_interface::CommandInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}

return_type FakeRobot::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");
  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type FakeRobot::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type FakeRobot::read()
{
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  // Update the wheel positions based on their velocities
  fl_wheel_.pos += fl_wheel_.vel * deltaSeconds;
  fr_wheel_.pos += fr_wheel_.vel * deltaSeconds;
  
  bl_wheel_.pos += bl_wheel_.vel * deltaSeconds;
  br_wheel_.pos += br_wheel_.vel * deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type FakeRobot::write()
{
  // Set the wheel velocities to directly match what is commanded
  fl_wheel_.vel = fl_wheel_.cmd;
  fr_wheel_.vel = fr_wheel_.cmd;
  
  bl_wheel_.vel = bl_wheel_.cmd;
  br_wheel_.vel = br_wheel_.cmd;

  return return_type::OK;  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  FakeRobot,
  hardware_interface::SystemInterface
)
