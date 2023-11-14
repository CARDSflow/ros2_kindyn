/*
    BSD 3-Clause License
    Copyright (c) 2018, Roboy
            All rights reserved.
    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.
    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    author: Simon Trendel ( st@gi.ai ), 2018
    description: A Torque controller for joint position targets using PD control
*/

#include <type_traits>
// #include <controller_interface/controller.h>
#include "controller_interface/controller_interface.hpp"

// #include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/loaned_command_interface.hpp>// not sure

#include <pluginlib/class_list_macros.hpp>
// #include "kindyn/robot.hpp"
#include "kindyn/controller/cardsflow_command_interface.hpp"
// #include <roboy_simulation_msgs/ControllerType.h>
#include "roboy_simulation_msgs/msg/controller_type.hpp"
#include <std_msgs/msg/float32.hpp>
// #include <roboy_control_msgs/SetControllerParameters.h>
#include "roboy_control_msgs/srv/set_controller_parameters.hpp"

using namespace std;

class TorquePositionController : public controller_interface::ControllerInterface,  rclcpp::Node {
public:
    /**
     * Constructor
     */
    TorquePositionController():Node("TorquePositionController") {};

    /**
     * Initializes the controller. Will be call by controller_manager when loading this controller
     * @param hw pointer to the hardware interface
     * @param n the nodehandle
     * @return success
     */
    // bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n) {

    bool init(hardware_interface::CardsflowCommandInterface *hw, rclcpp::Node::SharedPtr node) {
        node_ = node;
        // get joint name from the parameter server
        if (!node_->get_parameter("joint", joint_name)) {
             RCLCPP_ERROR(node_->get_logger(), "No joint given (namespace: %s)",std::string(node_->get_namespace()));
            return false;
        }

        // spinner.reset(new ros::AsyncSpinner(0));
        // spinner->start();
        // controller_state = nh.advertise<roboy_simulation_msgs::ControllerType>("/controller_type",1);
        controller_state = node_->create_publisher<roboy_simulation_msgs::msg::ControllerType>("/controller_type", 1);
        rclcpp::Rate r(10);
        while(controller_state->get_subscription_count()==0) // we wait until the controller state is available
            r.sleep();

        // joint = hw->getHandle(joint_name); // throws on failure

        // joint_command = nh.subscribe((joint_name+"/target").c_str(),1,&TorquePositionController::JointCommand, this);
        joint_command = node_->create_subscription<std_msgs::msg::Float32>(
            std::string(joint_name + "/target"),
            10,
            std::bind(&TorquePositionController::JointCommand,this,std::placeholders::_1)
        );



        // controller_parameter_srv = nh.advertiseService((joint_name+"/params").c_str(),& TorquePositionController::setControllerParameters, this);
        controller_parameter_srv = node_->create_service<roboy_control_msgs::srv::SetControllerParameters>(
            std::string(joint_name+"/params"),
            std::bind(&TorquePositionController::setControllerParameters,
            this, std::placeholders::_1, std::placeholders::_2)
        );
        return true;
    }

    /**
     * Called regularily by controller manager. The torque for a joint wrt to a PD controller on the joint target
     * position is calculated.
     * @param time current time
     * @param period period since last control
     */
    void update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        double q = joint.getPosition();
        double qd = joint.getVelocity();
        double p_error = q - q_target;
        double cmd = Kp * p_error + Kd *qd;

        //?????

        // joint.setCommand(cmd);
//        ROS_INFO_THROTTLE(1, "%s target %lf current %lf", joint_name.c_str(), q_target, q);
    }

    /**
     * Called by controller manager when the controller is about to be started
     * @param time current time
     */
    void starting(const rclcpp::Time& time) {
         RCLCPP_WARN(node_->get_logger(),"torque position controller started for %s", joint_name.c_str());
        roboy_simulation_msgs::msg::ControllerType msg;
        msg.joint_name = joint_name;
        msg.type = CARDSflow::ControllerType::torque_position_controller;
        controller_state->publish(msg);
    }

    /**
     * Called by controller manager when the controller is about to be stopped
     * @param time current time
     */
    void stopping(const rclcpp::Time& time) { RCLCPP_WARN(node_->get_logger(), "cable length controller stopped for %s", joint_name.c_str());}

    /**
     * Joint position command callback for this joint
     * @param msg joint position target in radians
     */
    void JointCommand(const std_msgs::msg::Float32::SharedPtr msg){
        q_target = msg->data;
    }

    /**
     * Controller Parameters service
     * @param req requested gains
     * @param res success
     * @return success
     */
    bool setControllerParameters( roboy_control_msgs::srv::SetControllerParameters::Request::SharedPtr req ,
                                  roboy_control_msgs::srv::SetControllerParameters::Response::SharedPtr res){
        Kp = req->kp;
        Kd = req->kd;
        res->success = true;
        return true;
    }
private:
    double q_target = 0; /// joint position target
    double Kp = 3000, Kd = 5; /// PD gains
    // ros::NodeHandle nh; /// ROS nodehandle
    rclcpp::Node::SharedPtr node_;/// ROS2 node
    // ros::Publisher controller_state; /// publisher for controller state
    rclcpp::Publisher<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_state;/// publisher for controller state
    // ros::ServiceServer controller_parameter_srv; /// service for controller parameters
    rclcpp::Service<roboy_control_msgs::srv::SetControllerParameters>::SharedPtr controller_parameter_srv; /// service for controller parameters
    // boost::shared_ptr<ros::AsyncSpinner> spinner;
    
    //？？
    // hardware_interface::JointHandle joint;
    hardware_interface::CardsflowHandle joint; /// cardsflow joint handle for access to joint/cable model state

    // ros::Subscriber joint_command; /// joint command subscriber
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_command; /// joint command subscriber
    
    string joint_name; /// name of the controlled joint
};
// PLUGINLIB_EXPORT_CLASS(TorquePositionController, controller_interface::ControllerBase);
