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
    description: A Cable length controller for joint position targets using PD control
*/

#include <type_traits>
// #include <controller_interface/controller.h>
#include "controller_interface/controller_interface.hpp"

// #include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/loaned_command_interface.hpp>

#include <pluginlib/class_list_macros.hpp>

// #include "kindyn/robot.hpp"

#include "kindyn/controller/cardsflow_state_interface.hpp"
#include "kindyn/controller/cardsflow_command_interface.hpp"

// #include <roboy_simulation_msgs/ControllerType.h>
#include "roboy_simulation_msgs/msg/controller_type.hpp"
// #include <std_msgs/Float32.h>
#include <std_msgs/msg/float32.hpp>
// #include <roboy_control_msgs/SetControllerParameters.h>
#include "roboy_control_msgs/srv/set_controller_parameters.hpp"


using namespace std;

// class CableLengthVelocityController : public controller_interface::Controller<hardware_interface::CardsflowCommandInterface> {
class CableLengthVelocityController : public controller_interface::ControllerInterface, rclcpp::Node {
public:
    /**
     * Constructor
     */
    CableLengthVelocityController():Node("CableLengthVelocityController"){};

    /**
     * Initializes the controller. Will be call by controller_manager when loading this controller
     * @param hw pointer to the hardware interface
     * @param node the node
     * @return success
     */
    bool init(hardware_interface::CardsflowCommandInterface *hw, rclcpp::Node::SharedPtr node) {
        node_ = node;
        // get joint name from the parameter server
        if (!node_->get_parameter("joint", joint_name)) {
            //ROS_ERROR   
            RCLCPP_ERROR(node_->get_logger(), "No joint given (namespace: %s)", std::string(node_->get_namespace()));
            return false;
        }

        // spinner.reset(new ros::AsyncSpinner(0));
        // spinner->start();
        controller_state = node_->create_publisher<roboy_simulation_msgs::msg::ControllerType>("/controller_type", 1);
        rclcpp::Rate r(10);

        while(controller_state->get_subscription_count()==0) // we wait until the controller state is available
            r.sleep();

        // joint = hw->getHandle(joint_name); // throws on failure

        joint_index = joint.getJointIndex();
        last_update = node_->now();

        // controller_statee = this->create_publisher<roboy_simulation_msgs::msg::ControllerType>("/controller_type", 1);
        //joint_command = node_->create_subscription<std_msgs::msg::Float32>(std::string(joint_name+"/target"), 1, std::bind(&CableLengthVelocityController::JointVelocityCommand, this, std::placeholders::_1));
        joint_command = [this]() {
            return node_->create_subscription<std_msgs::msg::Float32>(
                std::string(joint_name+"/target"), 1,
                [this](const std_msgs::msg::Float32::SharedPtr msg) {
                    this->JointVelocityCommand(msg);
                }
            );
        }();

        
        // joint_command = node_->create_subscription<std_msgs::msg::Float32>("test", 10, std::bind(&CableLengthVelocityController::JointVelocityCommand, this, std::placeholders::_1));
        // joint_command = node_->create_subscription<std_msgs::msg::Float32>((joint_name+"/target").c_str(),1,
        //                                         std::bind(&CableLengthVelocityController::JointVelocityCommand, this, std::placeholders::_1));
        // joint_command = node_->create_subscription<std_msgs::msg::Float32>(joint_name + "/target", 10, 
        // std::bind(&CableLengthVelocityController::JointVelocityCommand, this, std::placeholders::_1));
        
        controller_parameter_srv = [this](){
            return node_->create_service<roboy_control_msgs::srv::SetControllerParameters>(
                std::string(joint_name + "/params"),
                [this](roboy_control_msgs::srv::SetControllerParameters::Request ::SharedPtr req,
                        roboy_control_msgs::srv::SetControllerParameters::Response ::SharedPtr res){
                    this->setControllerParameters(req,res);
                }
            );
            }();

        // controller_parameter_srv = node_->create_service<roboy_control_msgs::srv::SetControllerParameters>
        // (joint_name + "/params", 
        // std::bind(&CableLengthVelocityController::setControllerParameters,
        //  this, std::placeholders::_1, std::placeholders::_2));  
        return true;
    }

    /**
     * Called regularily by controller manager. The length change of the cables wrt to a PD controller on the joint target
     * position is calculated.
     * @param time current time
     * @param period period since last control
     */
    void update(const rclcpp::Time &time, const rclcpp::Duration &period) {
        double qd = joint.getVelocity();
        double qd_target = joint.getJointVelocityCommand();
        MatrixXd L = joint.getL();
        double p_error = qd - qd_target;
        // we use the joint_index column of the L matrix to calculate the result for this joint only
        VectorXd ld = L.col(joint_index) * (Kd * (p_error - p_error_last)/period.seconds() + Kp * p_error);
//        ROS_INFO_STREAM_THROTTLE(1, ld.transpose());
        joint.setMotorCommand(ld);
        p_error_last = p_error;
        last_update = time;
    }

    /**
     * Called by controller manager when the controller is about to be started
     * @param time current time
     */
    void starting(const rclcpp::Time& time) {
        RCLCPP_WARN(node_->get_logger(), "cable velocity controller started for %s with index %d", joint_name.c_str(), joint_index);
        roboy_simulation_msgs::msg::ControllerType msg;
        msg.joint_name = joint_name;
        msg.type = CARDSflow::ControllerType::cable_length_controller;
        controller_state->publish(msg);
    }
    /**
     * Called by controller manager when the controller is about to be stopped
     * @param time current time
     */
    void stopping(const rclcpp::Time& time) { 
        RCLCPP_WARN(node_->get_logger(),"cable velocity controller stopped for %s", joint_name.c_str());}

    /**
     * Joint position command callback for this joint
     * @param msg joint position target in radians
     */
    // void JointVelocityCommand(const std_msgs::msg::Float32 &msg){
    //     joint.setJointVelocityCommand(msg.data);
    // }
    void JointVelocityCommand(const std_msgs::msg::Float32::SharedPtr msg) {
        joint.setJointVelocityCommand(msg->data);
    }


    /**
     * Controller Parameters service
     * @param req requested gains
     * @param res success
     * @return success
     */
    bool setControllerParameters( roboy_control_msgs::srv::SetControllerParameters::Request ::SharedPtr req,
                                  roboy_control_msgs::srv::SetControllerParameters::Response ::SharedPtr res){
        Kp = req->kp;
        Kd = req->kd;
        res->success = true;
        return true;
    }
private:

     rclcpp::Publisher<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_statee;


    double Kp = 0.1, Kd = 0; /// PD gains
    double p_error_last = 0; /// last error
    rclcpp::Node::SharedPtr node_; /// ROS2 node
    rclcpp::Publisher<roboy_simulation_msgs::msg::ControllerType>::SharedPtr controller_state;/// publisher for controller state
    rclcpp::Service<roboy_control_msgs::srv::SetControllerParameters>::SharedPtr controller_parameter_srv; /// service for controller parameters
    //boost::shared_ptr<ros::AsyncSpinner> spinner; /// ROS async spinner
    hardware_interface::CardsflowHandle joint; /// cardsflow joint handle for access to joint/cable model state
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr joint_command; /// joint command subscriber
    
    string joint_name; /// name of the controlled joint
    int joint_index; /// index of the controlled joint in the robot model
    rclcpp::Time last_update; /// time of last update
};

//PLUGINLIB_EXPORT_CLASS(CableLengthVelocityController, controller_interface::ControllerBase);
