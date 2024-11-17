#include "diffdrive_arduino/diffdrive_arduino.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = std::chrono::system_clock::now();

  // Update with four wheel names
  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];
  
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  //Load PID values
  cfg_.k_p = std::stof(info_.hardware_parameters["k_p"]);
  cfg_.k_i = std::stof(info_.hardware_parameters["k_i"]);
  cfg_.k_d = std::stof(info_.hardware_parameters["k_d"]);
  cfg_.k_o = std::stof(info_.hardware_parameters["k_o"]);
  // Set up the wheels
  fl_wheel_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  fr_wheel_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  bl_wheel_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  br_wheel_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  // Set up the Arduino
  arduino_.setup(cfg_.device, cfg_.baud_rate, cfg_.timeout);  

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fl_wheel_.name, hardware_interface::HW_IF_POSITION, &fl_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(fr_wheel_.name, hardware_interface::HW_IF_POSITION, &fr_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(bl_wheel_.name, hardware_interface::HW_IF_POSITION, &bl_wheel_.pos));

  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.vel));
  state_interfaces.emplace_back(hardware_interface::StateInterface(br_wheel_.name, hardware_interface::HW_IF_POSITION, &br_wheel_.pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(fl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(fr_wheel_.name, hardware_interface::HW_IF_VELOCITY, &fr_wheel_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(bl_wheel_.name, hardware_interface::HW_IF_VELOCITY, &bl_wheel_.cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(br_wheel_.name, hardware_interface::HW_IF_VELOCITY, &br_wheel_.cmd));

  return command_interfaces;
}

return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  arduino_.sendEmptyMsg();
  // Set PID values for the motors (can be tuned as needed)
  arduino_.setPidValues(cfg_.k_p, cfg_.k_i, cfg_.k_d, cfg_.k_o);
  status_ = hardware_interface::status::STARTED;
  // Reset encoder counts to zero for all wheels
  fl_wheel_.enc = 0;
  fr_wheel_.enc = 0;
  bl_wheel_.enc = 0;
  br_wheel_.enc = 0;

  // Also set positions and velocities to zero
  fl_wheel_.pos = 0.0;
  fr_wheel_.pos = 0.0;
  bl_wheel_.pos = 0.0;
  br_wheel_.pos = 0.0;

  fl_wheel_.vel = 0.0;
  fr_wheel_.vel = 0.0;
  bl_wheel_.vel = 0.0;
  br_wheel_.vel = 0.0;
  
  arduino_.resetEncoder();

  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;
  arduino_.resetEncoder();
  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{
  // Calculate time delta
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;

  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  // Read encoder values for all four wheels
  long frontLeftEnc, frontRightEnc, backLeftEnc, backRightEnc;
  arduino_.readEncoderValues(frontLeftEnc, frontRightEnc, backLeftEnc, backRightEnc);

  // Update wheel encoder values
  fl_wheel_.enc = frontLeftEnc;
  fr_wheel_.enc = frontRightEnc;
  bl_wheel_.enc = backLeftEnc;
  br_wheel_.enc = backRightEnc;

  // Calculate position and velocity for each wheel
  double pos_prev = fl_wheel_.pos;
  fl_wheel_.pos = fl_wheel_.calcEncAngle();
  fl_wheel_.vel = (fl_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = fr_wheel_.pos;
  fr_wheel_.pos = fr_wheel_.calcEncAngle();
  fr_wheel_.vel = (fr_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = bl_wheel_.pos;
  bl_wheel_.pos = bl_wheel_.calcEncAngle();
  bl_wheel_.vel = (bl_wheel_.pos - pos_prev) / deltaSeconds;

  pos_prev = br_wheel_.pos;
  br_wheel_.pos = br_wheel_.calcEncAngle();
  br_wheel_.vel = (br_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write()
{
  if (!arduino_.connected())
  {
    return return_type::ERROR;
  }

  // Send motor commands for all four wheels
  arduino_.setMotorValues(
    fl_wheel_.cmd / fl_wheel_.rads_per_count / cfg_.loop_rate,
    fr_wheel_.cmd / fr_wheel_.rads_per_count / cfg_.loop_rate,
    bl_wheel_.cmd / bl_wheel_.rads_per_count / cfg_.loop_rate,
    br_wheel_.cmd / br_wheel_.rads_per_count / cfg_.loop_rate
  );

  return return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
