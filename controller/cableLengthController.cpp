#include "ros2_control_kindyn/cableLengthController.hpp"

namespace ros2_control_kindyn
{

CableLengthController::CableLengthController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn CableLengthController::on_init()
{
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ = auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  state_interface_types_   = auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CableLengthController::command_interface_configuration() const
{
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start command interface configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}}; // config_type::ALL

  conf.names.reserve(joint_names_.size() * command_interface_types_.size()); /// 13 joints and 1 command_interface_type: position
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : command_interface_types_)
    {
      // RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "Added %s/%s to InterfaceConfiguration", joint_name.c_str(), interface_type.c_str());
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::InterfaceConfiguration CableLengthController::state_interface_configuration() const
{
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start state interface configuration");
  controller_interface::InterfaceConfiguration conf = {config_type::INDIVIDUAL, {}}; // config_type::ALL

  conf.names.reserve(joint_names_.size() * state_interface_types_.size()); /// 13 joints and 1 state_interface_type: position
  for (const auto & joint_name : joint_names_)
  {
    for (const auto & interface_type : state_interface_types_)
    {
      // RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "Added %s/%s to StateInterfaceConfiguration", joint_name.c_str(), interface_type.c_str());
      conf.names.push_back(joint_name + "/" + interface_type);
    }
  }

  return conf;
}

controller_interface::CallbackReturn CableLengthController::on_configure( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_configure");
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CableLengthController::on_activate( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_activate");
  
  // for (auto & interface : command_interfaces_)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "command_interfaces_[i].get_interface_name() = %s", interface.get_interface_name().c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "command_interfaces_[i].get_value() = %f", interface.get_value());
  // }

  // for (hardware_interface::LoanedStateInterface & interface : state_interfaces_)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "state_interfaces_[i].get_interface_name() = %s", interface.get_interface_name().c_str());
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "state_interfaces_[i].get_value() = %f", interface.get_value());
  // }

  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CableLengthController::on_deactivate( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_deactivate");
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CableLengthController::on_cleanup( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_cleanup");
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CableLengthController::on_error( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_error");
  return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn CableLengthController::on_shutdown( const rclcpp_lifecycle::State & previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "start on_shutdown");
  return CallbackReturn::SUCCESS;
}


controller_interface::return_type CableLengthController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
 
  // for (uint i = 0; i < state_interfaces_.size(); i++)
  // {
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "state_interfaces_[%d].get_value() = %f", i, state_interfaces_[i].get_value());
  // }
  // for (uint i = 0; i < command_interfaces_.size(); i++)
  // {
  //   command_interfaces_[i].set_value(command_interfaces_[i].get_value() - 0.001);
  //   RCLCPP_INFO(rclcpp::get_logger("CableLengthController"), "command_interfaces_[%d].get_value() = %f", i, command_interfaces_[i].get_value());
  // }

  return controller_interface::return_type::OK;
}


}  // namespace ros2_control_kindyn

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_kindyn::CableLengthController, controller_interface::ControllerInterface)
