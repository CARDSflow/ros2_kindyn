#ifndef ROS2_CONTROL_KINDYN__CABLE_LENGTH_CONTROLLER_HPP_
#define ROS2_CONTROL_KINDYN__CABLE_LENGTH_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <limits>

#include "rclcpp/macros.hpp"
#include "ros2_control_kindyn/visibility_control.h"
#include "ros2_control_kindyn/cardsflow_command_interface.hpp"
#include "ros2_control_kindyn/cardsflow_state_interface.hpp"

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "roboy_simulation_msgs/msg/controller_type.hpp"


using config_type = controller_interface::interface_configuration_type;

namespace ros2_control_kindyn
{

class CableLengthController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CableLengthController);

  CONTROLLER_INTERFACE_PUBLIC
  CableLengthController();
  
  /**
   * allocates memory that exists for the lifetime of the controller
   * here the joint_names_, command_interfaces_types and state_interface_types_ will be initialised.
   * Will be called by controller_manager when loading this controller
   */
  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::CallbackReturn on_init() override;
  /**
   * adds required command interface to `conf` by specifying their names and interface types.
   * returns a list of InterfaceConfiguration objects to indicate which command interfaces the controller needs to operate.
   * The command interfaces are uniquely identified by their name and interface type.
   * If a requested interface is not offered by a loaded hardware interface, then the controller will fail.
   */
  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

   /**
    * adds required state interface to `conf` by specifying their names and interface types.
    * returns a list of InterfaceConfiguration objects to indicate which state interfaces the controller needs to operate.
    */
  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  /**
   * read controller inputs values from state interfaces
   * calculate controller output values and write them to command interfaces
   * a realtime buffer is used to transfer the message from the subscriber to the realtime control loop thread.
   */
  CONTROLLER_INTERFACE_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;


  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  std::vector<std::string> joint_names_;             /// stores all needed joint names like shoulder_left_axis0. They are defined in robody_controllers.yaml
  std::vector<std::string> command_interface_types_; /// stores all needed interface types like position or velocity. They are defined in robody_controllers.yaml
  std::vector<std::string> state_interface_types_;   /// stores all needed interface types like position or velocity. They are defined in robody_controllers.yaml

};

}  // namespace ros2_control_kindyn

#endif  // ROS2_CONTROL_KINDYN__CABLE_LENGTH_CONTROLLER_HPP_
