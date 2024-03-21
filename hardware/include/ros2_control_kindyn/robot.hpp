//
// Created by roboy on 30.02.24.
//

#ifndef ROS2_CONTROL_KINDYN__ROBOT_HPP_
#define ROS2_CONTROL_KINDYN__ROBOT_HPP_

#pragma once

#include "ros2_control_kindyn/kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "ros2_control_kindyn/cable.hpp"
#include "ros2_control_kindyn/EigenExtension.hpp"
#include "ros2_control_kindyn/cardsflow_state_interface.hpp"
#include "ros2_control_kindyn/cardsflow_command_interface.hpp"
#include "ros2_control_kindyn/visibility_control.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <roboy_simulation_msgs/msg/tendon.hpp>
#include "roboy_simulation_msgs/msg/controller_type.hpp"
#include "roboy_simulation_msgs/msg/joint_state.hpp"
#include "roboy_middleware_msgs/srv/forward_kinematics.hpp"
#include "roboy_middleware_msgs/srv/inverse_kinematics.hpp"
// #include <roboy_middleware_msgs/srv/inverse_kinematics_multiple_frames.hpp>
#include "roboy_middleware_msgs/msg/motor_command.hpp"
#include "roboy_middleware_msgs/msg/motor_status.hpp"
#include "roboy_control_msgs/msg/end_effector.hpp" // #include <roboy_control_msgs/MoveEndEffectorAction.h>
#include "roboy_control_msgs/msg/strings.hpp"

// #include <tf2/LinearMath/Quaternion.h> // #include <tf/tf.h>
// tf2_ros and tf2_eigen nedded?
// #include "tf2_ros/transform_broadcaster.hpp"
// #include <tf2_ros/transform_listener.h>
// #include <tf2_eigen/tf2_eigen.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h> // #include <eigen_conversions/eigen_msg.h>

// #include <controller_manager/controller_manager.h>
// #include <controller_manager_msgs/LoadController.h>
// #include <hardware_interface/joint_state_interface.h>
// #include <hardware_interface/joint_command_interface.h>
// #include <hardware_interface/robot_hw.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include <common_utilities/rviz_visualization.hpp> // still in ROS1
// #include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>


namespace cardsflow {
  namespace kindyn {

    # define M_2PI 2*M_PI
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class HARDWARE_INTERFACE_PUBLIC RobotHardware : public hardware_interface::SystemInterface
    {
    public:
      /**
        * Constructor
        */
      RobotHardware();

      /**
        * Destructor
        */
      ~RobotHardware();

      /**
        * is called once during ros2_control initialization if the Robot was specified in the URDF
        * sets up the communication between the robot hardware and allocates memory dynamically
        * @param info structure with data from URDF.
        * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
        * \returns CallbackReturn::ERROR if any error happens or data are missing.
        */
      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      // TODO
      /// Exports all state interfaces for this hardware interface.
      /**
      * The state interfaces have to be created and transferred according
      * to the hardware info passed in for the configuration.
      *
      * Note the ownership over the state interfaces is transferred to the caller.
      *
      * \return vector of state interfaces
      */
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      // TODO
      /// Exports all command interfaces for this hardware interface.
      /**
      * The command interfaces have to be created and transferred according
      * to the hardware info passed in for the configuration.
      *
      * Note the ownership over the state interfaces is transferred to the caller.
      *
      * \return vector of command interfaces
      */
      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      /**
        * This is the read function and should implement reading the state of your robot.
        * It is called during the main loop.
        * ros2_control loops over all hardware components and calls this read method.
        * It is responsible for updating the data values of the state_interfaces. 
        */
      virtual hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
      {
        RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 1.0, "reading virtual, " "you probably forgot to implement your own read function?!");
        return hardware_interface::return_type::OK;
      };

      /**
        * This is the write function and should implement writing commands to your robot
        * It is called after update in the realtime loop.
        * It is responsible for updating the data values of the command_interfaces.
        * It accesses data values pointer to by the exported CommandInterface objects sends them to the corresponding hardware.
        */
      virtual hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
      {
        RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 1.0, "writing virtual, " "you probably forgot to implement your own write function?!");
      //   // for (uint i = 0; i < hw_commands_dummy_.size(); i++) {
      //   //   hw_states_dummy_[i] = hw_commands_dummy_[i];
      //   // }
        return hardware_interface::return_type::OK;
      };

      /**
       * Updates the model
       */
      void update();


      // TODO: implement this function in side the cableLengthController when ROS2 Jazzy is released
      // controller_interface::return_type RobotHardware::controller_update(const rclcpp::Time & time, const rclcpp::Duration & period)
      void controller_update(hardware_interface::CardsflowHandle & joint, const rclcpp::Time & time, const rclcpp::Duration & period, size_t i);

   
      bool simulated = false;

    protected:

      // TODO: Store the real command for the simulated robot, not just dummy values
      std::vector<double> hw_states_dummy_;
      std::vector<double> hw_commands_dummy_;
      std::vector<hardware_interface::CardsflowStateHandle> hw_states_;
      std::vector<hardware_interface::CardsflowHandle> hw_commands_;

      void updatePublishers();
      void updateSubscribers();
      void publishViz();

      /**
        * Callback for the joint state of the robot. This can come from gazebo, the real robot or else where.
        * @param msg message containing joint_name/angle information
        */
      void JointState(const sensor_msgs::msg::JointState::SharedPtr msg);
      void JointTarget(const sensor_msgs::msg::JointState::SharedPtr msg);

      /**
        * Callback for controller type change. The controller type defines how the forwardKinematics function
        * integrates the robot states
        * @param msg message containing the joint_name/type pair
        */
      void controllerType(const roboy_simulation_msgs::msg::ControllerType::SharedPtr msg);
      void ZeroJoints(const roboy_control_msgs::msg::Strings::SharedPtr msg);
      bool FreezeService(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res);

      void print_vec(VectorXd vec, std::string name);

      cardsflow::kindyn::Kinematics kinematics;
      rclcpp::Node::SharedPtr node_; // ros::NodeHandlePtr nh; /// ROS node handle
      // boost::shared_ptr<ros::AsyncSpinner> spinner; /// async ROS spinner
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner;
      std::thread executor_thread;


      bool torque_position_controller_active = false;
      bool force_position_controller_active = false;
      bool cable_length_controller_active = false;
      // hardware_interface::CardsflowStateInterface cardsflow_state_interface; /// cardsflow state interface
      // hardware_interface::CardsflowCommandInterface cardsflow_command_interface; /// cardsflow command interface
      vector<int> controller_type; /// currently active controller type

      // ros::Publisher robot_state_pub, tendon_state_pub, tendon_ext_state_pub, joint_state_pub, cardsflow_joint_states_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_pub;
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_state_pub;
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_ext_state_pub; 
      rclcpp::Publisher<roboy_simulation_msgs::msg::JointState>::SharedPtr joint_state_pub;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cardsflow_joint_states_pub;
      // ros::Publisher robot_state_target_pub, tendon_state_target_pub, joint_state_target_pub; /// target publisher
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_target_pub;
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_state_target_pub;
      rclcpp::Publisher<roboy_simulation_msgs::msg::JointState>::SharedPtr joint_state_target_pub; 
      // ros::Subscriber controller_type_sub, joint_state_sub, floating_base_sub, interactive_marker_sub, joint_target_sub, zero_joints_sub; /// ROS subscribers
      rclcpp::Subscription<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_type_sub;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub;
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr floating_base_sub;
      //TODO rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr>::SharedPtr interactive_marker_sub;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub;
      rclcpp::Subscription<roboy_control_msgs::msg::Strings>::SharedPtr zero_joints_sub;
      // ros::ServiceServer ik_srv, ik_two_frames_srv, fk_srv, freeze_srv;
      rclcpp::Service<roboy_middleware_msgs::srv::InverseKinematics>::SharedPtr ik_srv;
      rclcpp::Service<roboy_middleware_msgs::srv::InverseKinematics>::SharedPtr ik_two_frames_srv;
      rclcpp::Service<roboy_middleware_msgs::srv::ForwardKinematics>::SharedPtr fk_srv;
      rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr freeze_srv;
      string topic_root;

      VectorXd q, qd, qdd; /// joint positon, velocity, acceleration
      VectorXd q_target, qd_target, qdd_target; /// joint positon, velocity, acceleration targets. Value is set in JointTarget
      VectorXd q_target_prev, qd_target_prev, qdd_target_prev; /// joint positon, velocity, acceleration targets
      VectorXd l_next, l_target;
      vector<VectorXd> Ld; // tendon velocity per endeffector
      vector<VectorXd> ld; /// tendon length changes for each controller

      bool first_update = true;
      rclcpp::Time last_visualization; /// timestamp for visualization at reasonable intervals
      vector<rclcpp::Time> last_update; /// TODO Move this to controller, used for the controller update
      vector<double> p_error_last; /// TODO Move this to controller, used for the controller update
      Eigen::IOFormat fmt; /// formator for terminal printouts
      bool external_robot_state; /// indicates if we get the robot state externally
      bool external_robot_target = false;
      bool debug_ = false;
      double k_dt = 0.005;
      VectorXd Kp_, Kd_;
      vector<double> param_kp, param_kd;
    };

  } // namespace kindyn
}  // namespace cardsflow

#endif  // ROS2_CONTROL_KINDYN__ROBOT_HPP_
