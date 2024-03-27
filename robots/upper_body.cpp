#include "ros2_control_kindyn/robot.hpp"
#include <thread>

// #include "ros2_common_utilities/CommonDefinitions.hpp" // TODO include is not working
#include "roboy_middleware_msgs/msg/motor_state.hpp" 
#include "roboy_middleware_msgs/msg/roboy_state.hpp"
#include "roboy_middleware_msgs/msg/motor_info.hpp"
#include "roboy_middleware_msgs/msg/motor_config.hpp"
#include "roboy_middleware_msgs/msg/system_status.hpp"
#include "roboy_middleware_msgs/msg/body_part.hpp" 
#include "roboy_simulation_msgs/msg/tendon.hpp"
#include "roboy_control_msgs/srv/set_controller_parameters.hpp" 
#include "roboy_middleware_msgs/srv/control_mode.hpp"
#include "roboy_middleware_msgs/srv/motor_config_service.hpp" 
#include "roboy_middleware_msgs/srv/set_strings.hpp"
#include <std_srvs/srv/empty.hpp>

#include "tf2/transform_datatypes.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h" 

#include "rclcpp/rclcpp.hpp"


using namespace std;

class UpperBody : public cardsflow::kindyn::RobotHardware {
private:
    rclcpp::Node::SharedPtr node_; /// ROS nodehandle
    rclcpp::Publisher<roboy_middleware_msgs::msg::MotorCommand>::SharedPtr motor_command; /// motor command publisher
    rclcpp::Publisher<roboy_middleware_msgs::msg::SystemStatus>::SharedPtr system_status_pub;
    rclcpp::Publisher<roboy_simulation_msgs::msg::Tendon>::SharedPtr tendon_motor_pub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr  joint_target_pub; 
    rclcpp::Subscription<roboy_middleware_msgs::msg::MotorState>::SharedPtr motor_state_sub;
    rclcpp::Subscription<roboy_middleware_msgs::msg::MotorInfo>::SharedPtr motor_info_sub;
    rclcpp::Subscription<roboy_middleware_msgs::msg::RoboyState>::SharedPtr roboy_state_sub;
    // vector<ros::ServiceServer> init_poses;
    rclcpp::Service<roboy_middleware_msgs::srv::SetStrings>::SharedPtr init_pose;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> spinner; // ros::AsyncSpinner *spinner;
    std::thread executor_thread;

    map<int,int> pos, initial_pos;
    map<string, rclcpp::Client<roboy_middleware_msgs::srv::MotorConfigService>::SharedPtr> motor_config;
    // map<string, rclcpp::Client<???>::SharedPtr> motor_control_mode; // not used
    map<string, rclcpp::Client<roboy_middleware_msgs::srv::ControlMode>::SharedPtr> control_mode;
    map<string, bool> motor_status_received;
    map<int, bool> communication_established; /// keeps track of communication quality for each motor
    map<int,float> l_offset, position, tendon_length, muscle_length_offset_correction;
    VectorXd l_current;
    map<string, vector<float>> integral, error_last;
    // std::shared_ptr<tf::TransformListener> listener; // TODO
    std::vector<string> body_parts = {"head", "shoulder_right", "shoulder_left", "wrist_right","wrist_left"};//, "shoulder_left"};//}, "elbow_left"};
    map<string, bool> init_called;
    std::vector<bool> initialized;
    std::shared_ptr<std::thread> system_status_thread;
    rclcpp::Time prev_roboy_state_time;
    enum BulletPublish {zeroes, current};

public:

    /**
     * Constructor
     */
    UpperBody(){
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "constructing UpperBody in empty constructor.");
        node_ = std::make_shared<rclcpp::Node>("upper_body");
        
        node_->declare_parameter("urdf_file_path", "");
        string urdf = "overwrite";
        node_->get_parameter("urdf_file_path", urdf);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "urdf: %s", urdf.c_str());

        node_->declare_parameter("cardsflow_xml", "");
        string cardsflow_xml = "overwrite";
        node_->get_parameter("cardsflow_xml", cardsflow_xml);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "cardsflow_xml: %s", cardsflow_xml.c_str());

        node_->declare_parameter("robot_model", "");
        string robot_model = "overwrite";
        node_->get_parameter("robot_model", robot_model);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "robot_model: %s", robot_model.c_str());

        node_->declare_parameter("debug", false);
        node_->get_parameter("debug", debug_);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "debug: %d", debug_);

        spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        spinner->add_node(node_); // Add the node node_ to the executor
        executor_thread = std::thread([this]() { spinner->spin(); });

        vector<string> joint_names(0);
        node_->declare_parameter("joint_names", joint_names);
        node_->get_parameter("joint_names", joint_names);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "joint_names[0]: %s", joint_names[0].c_str());
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "joint_names[1]: %s", joint_names[1].c_str());
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "joint_names[2]: %s", joint_names[2].c_str());

        node_->declare_parameter("external_robot_state", false);
        bool external_robot_state = false;
        node_->get_parameter("external_robot_state", external_robot_state);
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "external_robot_state: %d", external_robot_state);
        topic_root = "/roboy/" + robot_model + "/";

        // on_init(hardware_interface::HardwareInfo & info) // not needed anymore, is called by ros2 control

        // listener.reset(new tf::TransformListener);
        l_current.resize(kinematics.number_of_cables);
        l_current.setZero();

        // RobotHardware::update(); // not needed anymore: can only be called when on_init(hardware_interface::HardwareInfo & info) was called

        motor_state_sub = node_->create_subscription<roboy_middleware_msgs::msg::MotorState>(topic_root + "middleware/MotorState", 1, std::bind(&UpperBody::MotorState, this, std::placeholders::_1));
        roboy_state_sub = node_->create_subscription<roboy_middleware_msgs::msg::RoboyState>(topic_root + "middleware/RoboyState", 1, std::bind(&UpperBody::RoboyState, this, std::placeholders::_1));
        motor_info_sub  = node_->create_subscription<roboy_middleware_msgs::msg::MotorInfo> (topic_root + "middleware/MotorInfo",  1, std::bind(&UpperBody::MotorInfo,  this, std::placeholders::_1));

        for (auto body_part: body_parts) {
            init_called[body_part] = false;
            motor_config[body_part] = node_->create_client<roboy_middleware_msgs::srv::MotorConfigService>(topic_root + "middleware/"+body_part+"/MotorConfig");
            control_mode[body_part] = node_->create_client<roboy_middleware_msgs::srv::ControlMode>( topic_root + "middleware/ControlMode");//+body_part+"ControlMode");
        }

        motor_command = node_->create_publisher<roboy_middleware_msgs::msg::MotorCommand>(topic_root + "middleware/MotorCommand", 1);
        init_pose = node_->create_service<roboy_middleware_msgs::srv::SetStrings>(topic_root + "init_pose", std::bind(&UpperBody::initPose, this, std::placeholders::_1, std::placeholders::_2));       
        joint_target_pub = node_->create_publisher<sensor_msgs::msg::JointState>(topic_root + "simulation/joint_targets", 1);

        system_status_pub = node_->create_publisher<roboy_middleware_msgs::msg::SystemStatus>(topic_root + "control/SystemStatus", 1);
        system_status_thread = std::shared_ptr<std::thread>(new std::thread(&UpperBody::SystemStatusPublisher, this));
        system_status_thread->detach();

        tendon_motor_pub = node_->create_publisher<roboy_simulation_msgs::msg::Tendon>(topic_root + "control/tendon_state_motor", 1);

        // TODO
        // std::vector<rclcpp::Parameter> initialized_parameter{rclcpp::Parameter("initialized", init_called)};
        // node_->set_parameters(initialized_parameter)
        initialized.resize(body_parts.size());
        for (size_t i = 0; i < initialized.size(); i++) {
            initialized[i] = false;
        }

        declare_parameter_body_part_motor_ids();

        RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), "Finished setup");

    };

    ~UpperBody() {
        if (system_status_thread->joinable())
            system_status_thread->join();
        spinner->cancel();
        if (executor_thread.joinable())
            executor_thread.join();
        rclcpp::shutdown();
    }

    void SystemStatusPublisher() {
        rclcpp::Rate rate(100);
        while (rclcpp::ok()) {
            auto msg = roboy_middleware_msgs::msg::SystemStatus();
            msg.header.stamp = rclcpp::Clock().now();
            auto body_part = roboy_middleware_msgs::msg::BodyPart();
            for (auto part: body_parts) {
                body_part.name = part;
                body_part.status = !init_called[part];
                msg.body_parts.push_back(body_part);
            }
            system_status_pub->publish(msg);
            rate.sleep();
        }
    }


    bool initPose(roboy_middleware_msgs::srv::SetStrings::Request::SharedPtr req,
                  roboy_middleware_msgs::srv::SetStrings::Response::SharedPtr res) {
        res->result = true;

        if (find(req->strings.begin(), req->strings.end(), "all") != req->strings.end()) {
            req->strings = body_parts;
        }

        for (string body_part: req->strings) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "init called for: " << body_part);
            init_called[body_part] = false;
            auto r = initBodyPart(body_part);
            res->result = r && res->result;
        }

        return res->result;
    }

    bool initBodyPart(string name) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "initBodyPart: " << name);

        std::vector<long int> motor_ids = {};
        try {
            node_->declare_parameter(name+".motor_ids", motor_ids);
            node_->get_parameter(name+".motor_ids", motor_ids);
            RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "%s/motor_ids: [%ld, %ld, %ld, ...]", name.c_str(), motor_ids[0], motor_ids[1], motor_ids[2]);
        }
        catch (const std::exception&) {
            RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), "motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", name.c_str());
            return false;
        }
        int pwm;
        try {
            if (name == "wrist_left" || name == "wrist_right") {
                node_->declare_parameter("m3_pwm", 0);
                node_->get_parameter("m3_pwm", pwm);
                RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "pwm: %d", pwm);
            } else {
                node_->declare_parameter("pwm", 0);
                node_->get_parameter("pwm",pwm);
                RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "pwm: %d", pwm);
            }
        }
        catch (const std::exception&) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("UpperBody"), "rosparam pwm or init_m3_displacement is not set. will not init.");
            return false;
        }


        RCLCPP_INFO(rclcpp::get_logger("UpperBody"),"changing control mode of motors to PWM with %d",pwm);
        std::shared_ptr<roboy_middleware_msgs::srv::ControlMode::Request> msg; // = roboy_middleware_msgs::srv::ControlMode::Request();


        // if (name == "wrist_left" || name == "wrist_right") {
        //     msg->control_mode = DISPLACEMENT;
        // } else {
            msg->control_mode = 3; // CONTROL_MODES::DIRECT_PWM; // TODO CommonDefinitions.h from ros2_common_utilities
        // }
        // TODO: fix in plexus PWM direction for the new motorboard
        std::vector<float> set_points(motor_ids.size(), pwm);
        for (auto m: motor_ids) msg->global_id.push_back(m);
        msg->set_points = set_points;

        stringstream str1;
        for(size_t i = 0; i < msg->set_points.size(); i++) {
            int motor_id = motor_ids[i];

            str1 << msg->global_id[i] << "\t|\t" << msg->set_points[i] << endl;
        }


        RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), str1.str());

        rclcpp::Client<roboy_middleware_msgs::srv::ControlMode>::FutureAndRequestId result = control_mode[name]->async_send_request(msg);
        if (!result.valid()) {
            RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), "Changing control mode for %s didnt work", name.c_str());
            return false;
        }


        rclcpp::Time t0;
        t0 = rclcpp::Clock().now();
        double timeout = 0;
        // node_->get_parameter("timeout",timeout); // TODO where is this comming from?
        if(timeout==0) {
            int seconds = 5;
            while ((rclcpp::Clock().now() - t0).seconds() < 5) {
                RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 1000, "waiting %d", seconds--);
            }
        }else{
            int seconds = timeout;
            while ((rclcpp::Clock().now() - t0).seconds() < timeout) {
                RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 1000, "waiting %d", seconds--);
            }
        }
        motor_status_received[name] = true;
        if(!motor_status_received[name]) {
            RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), "did not receive motor status for %s, try again", name.c_str());
            return false;
        }

        stringstream str;
        str << "saving position offsets:" << endl << "motor id  |   position offset [ticks]  | length_sim[m] | length offset[m] | manual offset correction [m]" << endl;

        // for (int i = 0; i<motor_ids.size();i++) str << motor_ids[i] << ": " << position[motor_ids[i]] << ", ";
        // str << endl;


        // Make sure we get current actual joint state
        t0= rclcpp::Clock().now();
        int seconds = 1;
        while ((rclcpp::Clock().now() - t0).seconds() < 1) {
            RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 1000, "waiting %d for external joint state", seconds--);
        }


        // Get current tendon length
        kinematics.setRobotState(q, qd);
        kinematics.getRobotCableFromJoints(l_current);

        // Get manual offset correction (feature requested by Oxford to change muscles' carrying load)


        // Get the parameter from the ROS parameter server
        // node_->get_parameter("muscle_length_offset_correction", muscle_length_offset_correction);

        for (int id: motor_ids) {
            if (!node_->get_parameter("muscle_length_offset_correction/motor"+to_string(id), muscle_length_offset_correction[id])) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "Failed to get muscle_length_offset_correction parameter for motor " << id <<". Ignoring it.");
                muscle_length_offset_correction[id] = 0.0;
            } 
        }
        
        // if (!node_->get_parameter("muscle_length_offset_correction", muscle_length_offset_correction))
        // {
        //     RCLCPP_ERROR("Failed to get muscle_length_offset_correction parameter. Ignoring it.");
        //     // for (int i = 0; i < motor_ids.size(); i++) {
        //     //     muscle_length_offset_correction[motor_ids[i]] = 0;
        //     // }
        // }

         // Check if each motor ID is in the dictionary, and if not, add it with a value of 0
        // for (int id : motor_ids)
        // {
        //     if (muscle_length_offset_correction.find(id) == muscle_length_offset_correction.end())
        //     {
        //         muscle_length_offset_correction[id] = 0.0;
        //     }
        // }


        for (size_t i = 0; i < motor_ids.size(); i++) {
            int motor_id = motor_ids[i];
            RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), name << " info print");
            l_offset[motor_id] = l_current[motor_id] + position[motor_id] + muscle_length_offset_correction[motor_id];
            str << motor_id << "\t|\t" << position[motor_id] << "\t|\t" << l_current[motor_id] << "\t|\t"
                << l_offset[motor_id] << "\t|\t"
                << muscle_length_offset_correction[motor_id] << endl;
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), str.str());

        RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), "changing control mode of %s to POSITION" << name);

        std::shared_ptr<roboy_middleware_msgs::srv::ControlMode::Request> msg1;
        msg1->control_mode = 0; // ENCODER0_POSITION; // TODO CommonDefinitions.h from ros2_common_utilities
        for (int id: motor_ids) {
            msg1->global_id.push_back(id);
            msg1->set_points.push_back(position[id]);
        }

        rclcpp::Client<roboy_middleware_msgs::srv::ControlMode>::FutureAndRequestId result1 = control_mode[name]->async_send_request(msg1);
        if (!result1.valid()) { // if (!control_mode[name].call(msg1)) {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("UpperBody"), "Changing control mode for %s didnt work" << name);
            return false;
        }

        vector<float> _integral(motor_ids.size(), 0);
        vector<float> _error(motor_ids.size(), 0);
        integral[name] = _integral;
        error_last[name] = _error;

        update();

        // node_->declare_parameter("external_robot_state", false);
        bool external_robot_state = false;
        node_->get_parameter("external_robot_state", external_robot_state);
        if(external_robot_state) {
            // Set current state to bullet
            publishBulletTarget(name, BulletPublish::current);

            t0 = rclcpp::Clock().now();
            int seconds = 3;
            while ((rclcpp::Clock().now() - t0).seconds() < 3) {
                RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 1000, "waiting %d for setting bullet", seconds--);
            }

            // Move back to zero position
            publishBulletTarget(name, BulletPublish::zeroes);
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), "%s pose init done" << name);
        init_called[name] = true;
        // node_->setParam("initialized", init_called); // TODO

        return true;
    }

    /**
     * Publish Target point to bullet
     * @param body_part
     * @param zeroes_or_current will publish either "zeroes" or "current" as targets to Bullet
     */
    void publishBulletTarget(string body_part, BulletPublish zeroes_or_current){

        sensor_msgs::msg::JointState target_msg;

        // set respecitve body part joint targets to 0
        string endeffector;
        node_->declare_parameter(body_part+"/endeffector", endeffector);
        node_->get_parameter(body_part+"/endeffector", endeffector);
        if ( !endeffector.empty() ) {
            vector<string> ik_joints;
            node_->declare_parameter(endeffector + "/joints", ik_joints);
            node_->get_parameter((endeffector + "/joints"), ik_joints);
            if (ik_joints.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), 
                        "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                        body_part.c_str());
            }
            else {

                for (auto joint: ik_joints) {
                    int joint_index = kinematics.GetJointIdByName(joint);
                    if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                        target_msg.name.push_back(joint);

                        if(zeroes_or_current == BulletPublish::zeroes) {
                            target_msg.position.push_back(0);
                            RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "Set target 0 for " << joint);
                        }else if(zeroes_or_current == BulletPublish::current){
                            target_msg.position.push_back(q[joint_index]);
                            RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "Set target " << q[joint_index] << " for " << joint);
                        }
                    }
                }

            }
        }

        joint_target_pub->publish(target_msg);
    }


    string findBodyPartByMotorId(int id) {
        string ret = "unknown";
        for (auto body_part: body_parts) {
            std::vector<long int> motor_ids;
            try {
//                mux.lock();
                node_->declare_parameter(body_part+".motor_ids", motor_ids);
                node_->get_parameter(body_part+".motor_ids", motor_ids);
                RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "%s/motor_ids: [%ld, %ld, %ld, ...]", body_part.c_str(), motor_ids[0], motor_ids[1], motor_ids[2]);
//                mux.unlock();
            }
            catch (const std::exception&) {
                RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), "motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part.c_str());
                return ret;
            }
            if (find(motor_ids.begin(), motor_ids.end(), id) != motor_ids.end()) {
                return body_part;
            }
        }
        RCLCPP_WARN_ONCE(rclcpp::get_logger("UpperBody"), "Seems like motor with id %d does not belong to any body part", id);
        return ret;
    }

    void MotorState(const roboy_middleware_msgs::msg::MotorState::SharedPtr msg){
        RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "executing MotorState function");
        prev_roboy_state_time = rclcpp::Clock().now();

        int i=0;
        for (auto id:msg->global_id) {
            position[id] = msg->encoder0_pos[i];
            tendon_length[id] = l_offset[id] - position[id];
            i++;
        }

    //    roboy_simulation_msgs::msg::Tendon tendon_msg;
    //    for (int j=0; j < tendon_length.size(); j++){
    //        tendon_msg.l.push_back(tendon_length[j]);
    //    }
    //    tendon_motor_pub->publish(tendon_msg);
    }

    void RoboyState(const roboy_middleware_msgs::msg::RoboyState::SharedPtr msg) {
    //    prev_roboy_state_time = rclcpp::Clock().now();
    }

    void MotorInfo(const roboy_middleware_msgs::msg::MotorInfo::SharedPtr msg){
        for (size_t i=0;i<msg->global_id.size();i++) {
            auto id = int(msg->global_id[i]);
            auto body_part = findBodyPartByMotorId(id);
//            RCLCPP_INFO_STREAM(rclcpp::get_logger("UpperBody"), body_part);
            if (body_part != "unknown") {
                if (msg->communication_quality[i] > 0 ) {
                    motor_status_received[body_part] = true;
                    //            communication_established[id] = true;
                }
                else {
                    if (body_part != "wrist_left" && body_part != "wrist_right")
                    {
                    //            communication_established[id] = false;
                    RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 10000, "Did not receive motor status for motor with id: %d. %s Body part is disabled.", id, body_part.c_str());

                        // TODO fix triceps
                        //if (id != 18 && body_part != "shoulder_right") {
                        if(init_called[body_part]) {
                            init_called[body_part] = false;
                            // nh->set_parameter("initialized", init_called); // TODO

                            // set respecitve body part joint targets to 0
                            string endeffector;
                            node_->get_parameter(body_part+"/endeffector", endeffector);
                            if ( !endeffector.empty() ) {
                                vector<string> ik_joints;
                                node_->get_parameter((endeffector + "/joints"), ik_joints);
                                if (ik_joints.empty()) {
                                    RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), 
                                            "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                                            body_part.c_str());
                                }
                                else {

                                    for (auto joint: ik_joints) {
                                        int joint_index = kinematics.GetJointIdByName(joint);
                                        if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
                                            q_target(joint_index) = 0;
                                            RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), "Set target 0 for " << joint);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
//      int i=0;
//      for (auto id:msg->global_id) {
//           RCLCPP_INFO_STREAM(rclcpp::get_logger(""), "ID: " << id);
//        auto body_part = findBodyPartByMotorId(id);
//        if (msg->communication_quality[i] > 0 && body_part != "unknown") {
//            motor_status_received[body_part] = true;
////            communication_established[id] = true;
//        }
//        else {
////            communication_established[id] = false;
//            RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 10000, "Did not receive %s's motor status for motor with id: %d. Body part is disabled.", (body_part.c_str(), id));
//            init_called[body_part] = false;
//        }
//        i++;
//      }
    }

    void declare_parameter_body_part_motor_ids() {
        for (auto body_part: body_parts) {
            std::vector<long int> motor_ids;
            node_->declare_parameter(body_part+".motor_ids", motor_ids);
        }
    }

    /**
     * Updates the robot model
     */
    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        RobotHardware::update();
        return hardware_interface::return_type::OK;
    };

    /**
     * Sends motor commands to the real robot
     */
    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override {
        // // check if plexus is alive
        // RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "rclcpp::Clock().now(): %ld, prev_roboy_state_time: %ld", rclcpp::Clock().now().nanoseconds(), prev_roboy_state_time.nanoseconds());
        // auto diff = rclcpp::Clock().now() - prev_roboy_state_time;
        // if (diff.seconds() > 1) {
        //     for (auto body_part: body_parts) {
        //         init_called[body_part] = false;
        //         // node_->set_parameter("initialized", init_called); // TODO
        //         // reset the joint targets
        //         q_target.setZero(); // set this to zero and stop moving if communication is lost
        //     }
        //     RCUTILS_LOG_ERROR_THROTTLE(RCUTILS_STEADY_TIME, 5000, "No messages from roboy_plexus. Will not be sending MotorCommand...");
        //     return hardware_interface::return_type::OK;;
        // }

        for (auto body_part: body_parts) {
            if (!init_called[body_part]) {
                // RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 10000, "%s was not initialized. skipping", body_part.c_str());
                // RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "body_part %s was not initialized. skipping", body_part.c_str());
            } else {
//             if(body_part == "shoulder_right"){
                std::vector<long int> motor_ids;
                try {
                    RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 2000, "body_part: %s", body_part.c_str());
                    node_->declare_parameter(body_part+"/motor_ids", motor_ids);
                    node_->get_parameter(body_part+"/motor_ids", motor_ids);
                    RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "%s/motor_ids: [%ld, %ld, %ld, ...]", body_part.c_str(), motor_ids[0], motor_ids[1], motor_ids[2]);
                }
                catch (const std::exception&) {
                    RCLCPP_ERROR(rclcpp::get_logger("UpperBody"), "motor ids for %s are not on the parameter server. check motor_config.yaml in robots.", body_part.c_str());
                }
                RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "writing2");

                stringstream str;
                std::shared_ptr<roboy_middleware_msgs::msg::MotorCommand> msg;
                msg->global_id = {};
                msg->setpoint = {};
                
                for (size_t i = 0; i < motor_ids.size(); i++) {
                    msg->global_id.push_back(motor_ids[i]);
                    auto setpoint = -l_next[motor_ids[i]] + l_offset[motor_ids[i]];
                    msg->setpoint.push_back(setpoint);
                    if (i==7) RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "i: 7, setpoint      = %f", setpoint);
                    // RCLCPP_INFO(rclcpp::get_logger("UpperBody"), "i: %ld, setpoint: %f", i, setpoint);
                }
                motor_command->publish(*msg);
                if(!str.str().empty())
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("UpperBody"), str.str());
            }
        }
        return hardware_interface::return_type::OK;
    };

};

// /**
//  * The controller manager update thread is not needed in ROS2. It was used to define how fast the controllers should run
//  * @param cm pointer to the controller manager
//  */
// void update(controller_manager::ControllerManager *cm) {
//     rclcpp::Time prev_time = rclcpp::Clock().now();
//     rclcpp::Rate rate(500); // changing this value affects the control speed of your running controllers
//     while (rclcpp::ok()) {
//         const rclcpp::Time time = rclcpp::Clock().now();
//         const rclcpp::Duration period = time - prev_time;
//         cm->update(time, period);
//         prev_time = time;
//         rate.sleep();
//     }
// }


int main(int argc, char *argv[]) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "starting main method of UpperBody");

    string robot_model(argv[1]);
    bool debug(argv[2]);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "launching " << robot_model);
    // if (!ros::isInitialized()) { // not available in ROS2
    //     int argc = 0;
    //     char **argv = NULL;
    //     ros::init(argc, argv, robot_model + "_upper_body"); // rclcpp::init(0, nullptr)
    // }

    rclcpp::Node::SharedPtr node_;
    string urdf, cardsflow_xml;
    
    if(node_->has_parameter("urdf_file_path") && node_->has_parameter("cardsflow_xml")) {
        node_->get_parameter("urdf_file_path", urdf);
        node_->get_parameter("cardsflow_xml", cardsflow_xml);
    }else {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "USAGE: rosrun kindyn test_robot path_to_urdf path_to_viapoints_xml");
        return 1;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\nurdf file path: %s\ncardsflow_xml %s", urdf.c_str(), cardsflow_xml.c_str());

    // UpperBody robot(); // not needed

    // controller_manager::ControllerManager cm(&robot);

    // if (node_->has_parameter("simulated")) {
    //   node_->get_parameter("simulated", robot.simulated);
    // }

    // thread update_thread(update, &cm);
    // update_thread.detach();

    // rclcpp::Rate rate(200);
    // while(rclcpp::ok()){
    //     robot.read();
    //     if (!robot.simulated)
    //       robot.write();
    //     rclcpp::spin_some(node_);
    //     rate.sleep();
    // }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "TERMINATING...");
    // update_thread.join();

    return 0;
}

#include "pluginlib/class_list_macros.hpp" 

PLUGINLIB_EXPORT_CLASS(
  UpperBody, hardware_interface::SystemInterface)