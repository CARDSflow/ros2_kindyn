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
#include "roboy_middleware_msgs/msg/motor_command.hpp"
#include "roboy_middleware_msgs/msg/motor_status.hpp"
#include "roboy_control_msgs/msg/end_effector.hpp" 
#include "roboy_control_msgs/msg/strings.hpp"

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2/convert.h> 

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// #include <common_utilities/rviz_visualization.hpp> // TODO include is not working

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
        * Is called once during ros2_control initialization if the Robot was specified in the URDF.
        * Sets up the communication between the robot hardware and allocates memory dynamically.
        * @param info structure with data from URDF.
        * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
        * \returns CallbackReturn::ERROR if any error happens or data are missing.
        */
      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      /**
      * The state interfaces have to be created and transferred according
      * to the hardware info passed in for the configuration.
      *
      * Note the ownership over the state interfaces is transferred to the caller.
      *
      * \return vector of state interfaces
      */
      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

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
        return hardware_interface::return_type::OK;
      };

      /**
       * Updates the model
       */
      void update();


      /* TODO: This function is the old update function from the ROS1 cableLengthController.
       * This should be moved back to the cableLengthController when ROS2 Jazzy is released because right now it is not possible to pass a class to the Humble controller.
       * This function should calculate the new state inside the controller and not inside the hardware interface like it is implemented right now.
       */
      void controller_update(hardware_interface::CardsflowHandle & joint, const rclcpp::Time & time, const rclcpp::Duration & period);
   
      bool simulated = false;

    protected:

      /*
       * Stores the positions of all joints to update Rviz.
       * This vector is passed to the controller.
       */
      std::vector<double> hw_states_fwd_pos_ctrl_;
      /*
       * This vector is passed to the controller but not really used.
       */
      std::vector<double> hw_commands_fwd_pos_ctrl_;
      /*
       * TODO: This vector should be passed to the CableLengthController but this is not yet possible.
       * This vector stores the joint state of all joints.
       */
      std::vector<hardware_interface::CardsflowStateHandle> hw_states_;
      /*
       * TODO: This vector should be passed to the CableLengthController but this is not yet possible.
       * This vector stores the joint command of all joints.
       */
      std::vector<hardware_interface::CardsflowHandle> hw_commands_;

      void updatePublishers();
      void updateSubscribers();
      void publishViz();

      /**
        * Callback for the joint state of the robot. This can come from gazebo, the real robot or else where.
        * @param msg message containing joint_name/angle information
        */
      void JointState(const sensor_msgs::msg::JointState::SharedPtr msg);

      /**
        * Callback for the targeted joint state of the robot. This can come from the controller or the programmer.
        * @param msg message containing joint_name/angle information
        */
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
      vector<int> controller_type; /// currently active controller type

      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_state_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_ext_state_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<roboy_simulation_msgs::msg::JointState>::SharedPtr joint_state_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cardsflow_joint_states_pub; /// ROS robot pose and tendon publisher
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_state_target_pub; /// target publisher
      rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_state_target_pub; /// target publisher
      rclcpp::Publisher<roboy_simulation_msgs::msg::JointState>::SharedPtr joint_state_target_pub; /// target publisher
      rclcpp::Subscription<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_type_sub; /// ROS subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub; /// ROS subscribers
      rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr floating_base_sub; /// ROS subscribers
      //TODO rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedbackConstPtr>::SharedPtr interactive_marker_sub; /// ROS subscribers
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_target_sub; /// ROS subscribers
      rclcpp::Subscription<roboy_control_msgs::msg::Strings>::SharedPtr zero_joints_sub; /// ROS subscribers
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
