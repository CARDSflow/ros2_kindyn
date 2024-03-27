//
// Created by roboy on 01.07.21. Updated 28.03.24
//

#include "ros2_control_kindyn/robot.hpp"

using namespace cardsflow::kindyn;

double wrap_pos_neg_pi(double angle)
{
    return fmod(angle + M_PI, M_2PI) - M_PI;
}

RobotHardware::RobotHardware() {
  RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "create RobotHardware Object");
  // rclcpp::init(0, nullptr); // 'rclcpp::ContextAlreadyInitialized'
  node_ = std::make_shared<rclcpp::Node>("CARDSflow_robot");
  auto parameter_client = std::make_shared<rclcpp::SyncParametersClient>(node_);

  spinner = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  spinner->add_node(node_); // Add the node node_ to the executor
  executor_thread = std::thread([this]() { spinner->spin(); });
}

RobotHardware::~RobotHardware() {
  spinner->cancel();
  if (executor_thread.joinable())
    executor_thread.join();
  rclcpp::shutdown();
}

void RobotHardware::updatePublishers() {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "advertising " << topic_root+"/robot_state");
  robot_state_pub            = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_root + "control/robot_state", 1);               // usd in publishViz
  tendon_state_pub           = node_->create_publisher<roboy_simulation_msgs::msg::Tendon>(topic_root + "control/tendon_state", 1);           // usd in publishViz
  tendon_ext_state_pub       = node_->create_publisher<roboy_simulation_msgs::msg::Tendon>(topic_root + "control/tendon_state_ext", 1);
  joint_state_pub            = node_->create_publisher<roboy_simulation_msgs::msg::JointState>(topic_root + "control/rviz_joint_states", 1);  // usd in publishViz
  cardsflow_joint_states_pub = node_->create_publisher<sensor_msgs::msg::JointState>(topic_root + "control/cardsflow_joint_states", 1);       // usd in publishViz
  robot_state_target_pub     = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_root + "control/robot_state_target", 1);        // usd in publishViz
  tendon_state_target_pub    = node_->create_publisher<roboy_simulation_msgs::msg::Tendon>(topic_root + "control/tendon_state_target", 1);
  joint_state_target_pub     = node_->create_publisher<roboy_simulation_msgs::msg::JointState>(topic_root + "control/joint_state_target", 1); 
}

void RobotHardware::updateSubscribers() {
  if (external_robot_state) {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Subscribing to external joint state");
    joint_state_sub   = node_->create_subscription<sensor_msgs::msg::JointState>              (topic_root + "/sensing/external_joint_states", 1, std::bind(&RobotHardware::JointState, this, std::placeholders::_1));
  }
  joint_target_sub    = node_->create_subscription<sensor_msgs::msg::JointState>              (topic_root + "control/joint_targets", 100, std::bind(&RobotHardware::JointTarget, this, std::placeholders::_1));
  controller_type_sub = node_->create_subscription<roboy_simulation_msgs::msg::ControllerType>("/controller_type", 100, std::bind(&RobotHardware::controllerType, this, std::placeholders::_1));
  freeze_srv          = node_->create_service     <std_srvs::srv::Trigger>                    (topic_root + "control/freeze", std::bind(&RobotHardware::FreezeService, this, std::placeholders::_1, std::placeholders::_2));
  zero_joints_sub     = node_->create_subscription<roboy_control_msgs::msg::Strings>          (topic_root + "control/zero_joints", 1, std::bind(&RobotHardware::ZeroJoints, this, std::placeholders::_1));
}

CallbackReturn RobotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  // SystemInterface::on_init(info) sets the parent parameter HardwareInfo info_ 
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  hw_states_fwd_pos_ctrl_.resize(info_.joints.size(), 0.0);   // forward position states   for the "cableLengthControler"
  hw_commands_fwd_pos_ctrl_.resize(info_.joints.size(), 0.0); // forward position commands for the "cableLengthControler"
  hw_states_.resize(info_.joints.size(), hardware_interface::CardsflowStateHandle());
  hw_commands_.resize(info_.joints.size(), hardware_interface::CardsflowHandle());
  p_error_last.resize(info_.joints.size(), 0.0);
  last_update.resize(info_.joints.size(), rclcpp::Clock().now());


  updatePublishers();
  fmt = Eigen::IOFormat(4, 0, " ", ";\n", "", "", "[", "]");
  node_->declare_parameter("external_robot_target", false);
  node_->get_parameter("external_robot_target", external_robot_target);
  node_->declare_parameter("external_robot_state", false);
  node_->get_parameter("external_robot_state", external_robot_state);

  vector<string> link_relation_names_default{"default"};
  node_->declare_parameter("link_relation_names", link_relation_names_default);
  node_->get_parameter("link_relation_names", kinematics.link_relation_name);
  for (string lr:kinematics.link_relation_name) {
    vector<string> lr_joint_names{"default"};
    node_->declare_parameter(("link_relation." + lr + ".joint_names"), lr_joint_names);
    node_->get_parameter(("link_relation." + lr + ".joint_names"), lr_joint_names);
    kinematics.joint_relation_name.push_back(lr_joint_names);
  }

  node_->declare_parameter("urdf_file_path", "default");
  string urdf_file_path = "overwrite";
  node_->get_parameter("urdf_file_path", urdf_file_path);

  node_->declare_parameter("viapoints_file_path", "default");
  string viapoints_file_path = "overwrite";
  node_->get_parameter("viapoints_file_path", viapoints_file_path);

  // order: sl0 sl1 sl2 el0 el1 wl0 wl1 wl2 h0 h1 h2 sr0 sr1 sr2 er0 er1 wr0 wr1 wr2
  vector<string> joint_names_ordered(info_.joints.size()); 
  for (uint i = 0; i < info_.joints.size(); i++) {
    joint_names_ordered[i] = info_.joints[i].name;
    RCLCPP_INFO(rclcpp::get_logger("RobotHardware"),"joint_names_ordered[%ld] = %s", i, joint_names_ordered[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "urdf_file_path: %s", urdf_file_path.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "viapoints_file_path: %s", viapoints_file_path.c_str());
  for(int i = 0; i < joint_names_ordered.size(); i++) {
      RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "joint_names_ordered[%d]: %s", i, joint_names_ordered[i].c_str());
  }
  kinematics.init(urdf_file_path, viapoints_file_path, joint_names_ordered);

  q.resize(kinematics.number_of_dofs);
  qd.resize(kinematics.number_of_dofs);
  qdd.resize(kinematics.number_of_dofs);

  RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "kinematics.number_of_dofs: %ld", kinematics.number_of_dofs);
  q_target.resize(kinematics.number_of_dofs);
  qd_target.resize(kinematics.number_of_dofs);
  qdd_target.resize(kinematics.number_of_dofs);

  q_target_prev.resize(kinematics.number_of_dofs);
  qd_target_prev.resize(kinematics.number_of_dofs);
  qdd_target_prev.resize(kinematics.number_of_dofs);

  l_next.resize(kinematics.number_of_cables);
  l_target.resize(kinematics.number_of_cables);
  ld.resize(kinematics.number_of_dofs);
  for (size_t i = 0; i < kinematics.number_of_dofs; i++) {
    ld[i].resize(kinematics.number_of_cables);
    ld[i].setZero();
  }

  q_target.setZero();
  qd_target.setZero();
  qdd_target.setZero();

  q_target_prev.setZero();
  qd_target_prev.setZero();
  qdd_target_prev.setZero();
  l_next.setZero();
  l_target.setZero();

  controller_type.resize(kinematics.number_of_cables, CARDSflow::ControllerType::cable_length_controller);

  /**
   * Register ROS control
   */
  // PD gains for cable length controller

  kinematics.joint_dt.resize(kinematics.number_of_dofs);
  Kp_.resize(kinematics.number_of_dofs);
  Kd_.resize(kinematics.number_of_dofs);
  param_kp.resize(kinematics.number_of_dofs);
  param_kd.resize(kinematics.number_of_dofs);

  vector<double> joint_dt_default{0.0};
  node_->declare_parameter("joint_dt", joint_dt_default);
  node_->get_parameter("joint_dt", kinematics.joint_dt);
    
  vector<double> joint_kp_default{0.0};
  node_->declare_parameter("joint_kp", joint_kp_default);
  node_->get_parameter("joint_kp", param_kp);

  vector<double> joint_kd_default{0.0};
  node_->declare_parameter("joint_kd", joint_kd_default);
  node_->get_parameter("joint_kd", param_kd);



  for (size_t joint = 0; joint < kinematics.number_of_dofs; joint++) {
    Kp_[joint] = param_kp[joint];
    Kd_[joint] = param_kd[joint];
  }


  for (size_t joint = 0; joint < kinematics.number_of_dofs; joint++) {
    RCLCPP_INFO(rclcpp::get_logger("RobotHardware"),"initializing controllers for joint %ld %s", joint, kinematics.joint_names[joint].c_str());
    // connect and register the cardsflow state interface
    hardware_interface::CardsflowStateHandle state_handle(kinematics.joint_names[joint], joint, &q[joint], &qd[joint],
                                                          &qdd[joint], &kinematics.L, &kinematics.M, &kinematics.CG
    );
    hw_states_[joint] = state_handle; // cardsflow_state_interface.registerHandle(state_handle);

    // connect and register the cardsflow command interface
    hardware_interface::CardsflowHandle pos_handle(state_handle, // cardsflow_state_interface.getHandle(kinematics.joint_names[joint]),
                                                    &q_target[joint], &qd_target[joint], &kinematics.torques[joint], &ld[joint],
                                                    &Kp_, &Kd_);
    hw_commands_[joint] = pos_handle; // cardsflow_command_interface.registerHandle(pos_handle);
  }

  /**
   * registerInterface was used in ROS1 but
   * it is not neccessary anymore because the controller update is now executed in this class.
   * later the hw_states_ and hw_commands_ will passed in a different way to the controller
   */
  // registerInterface(&cardsflow_command_interface);

  /**
   * Initialize at zero for simulation
   */
  if(!this->external_robot_state){
    q.setZero();
    qd.setZero();
    qdd.setZero();

    kinematics.setRobotState(q, qd);
    kinematics.updateJacobians();
  }

  try {
    last_visualization = rclcpp::Clock().now() - rclcpp::Duration(10, 0); // triggers immediate visualization in first iteratiom
  }
  catch(std::runtime_error& ex) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: [%s]", ex.what());
  }

  int k=0;
  vector<string> endeffectors_default{"default"};
  node_->declare_parameter("endeffectors", endeffectors_default);
  node_->get_parameter("endeffectors", kinematics.endeffectors);
  kinematics.endeffector_dof_offset.push_back(0);
  Ld.resize(kinematics.endeffectors.size());
  for (string ef:kinematics.endeffectors) {
    vector<string> ik_joints;
    node_->declare_parameter((ef + ".joints"), ik_joints);
    node_->get_parameter((ef + ".joints"), ik_joints);
    if (ik_joints.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "endeffector %s has no joints defined, check your endeffector.yaml or parameter server.  skipping...",
                  ef.c_str());
      continue;
    }
    kinematics.endeffector_index[ef] = k;
    kinematics.endeffector_number_of_dofs.push_back(ik_joints.size());
    if(k>0)
      kinematics.endeffector_dof_offset.push_back(kinematics.endeffector_dof_offset[k-1]+kinematics.endeffector_number_of_dofs[k-1]);

    Ld[k].resize(kinematics.number_of_cables);
    Ld[k].setZero();
    k++;
  }

  updateSubscribers();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_fwd_pos_ctrl_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_fwd_pos_ctrl_[i]));
  }
  return command_interfaces;
}

void RobotHardware::update() {

  // TODO: Run the below code in critical section to avoid Mutex with the JointState and PROBABLY the controller

  /**
    * Update Current robot state
    */
  kinematics.setRobotState(q, qd);

  /**
    * Update Jacobians matrix L with current joint state
    */
  kinematics.updateJacobians();

  /**
    * Visualization
    */
  publishViz();

  // ----------------------------------------------------------------------------

  /**
    * Update current tendon length from controller
    */

  // Doing the reduce_sum, the latter step of doing dot product in the controller
  for(size_t i = 0; i< kinematics.endeffectors.size();i++) {
    Ld[i].setZero();
    int dof_offset = kinematics.endeffector_dof_offset[i];
    for (size_t j = dof_offset; j < kinematics.endeffector_number_of_dofs[i] + dof_offset; j++) {
      Ld[i] -= ld[j];
    }
  }


  // ----------------------------------------------------------------------------
  // Do one step forward Kinematics with current tendon velocity Ld and current state
  vector<VectorXd> state_next = kinematics.oneStepForward(q, qd, Ld);

  if(!this->external_robot_state){
    for(size_t i=0;i<kinematics.number_of_joints;i++){
      q[i] = state_next[0][i];
      qd[i] = state_next[1][i];
      hw_states_fwd_pos_ctrl_[i] = q[i]; // This sets the angle for the Rviz visualization
    }
  }

  if(!simulated){
    kinematics.setRobotState(state_next[0], state_next[1]);
    kinematics.getRobotCableFromJoints(l_next);
  }

  print_vec(q, "q");

  for (size_t i = 0; i < hw_commands_.size(); i++) {
    controller_update(hw_commands_[i], rclcpp::Clock().now(), rclcpp::Clock().now() - last_update[i]);
  }

}

void RobotHardware::controller_update(hardware_interface::CardsflowHandle & joint, const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double q_ctrl = joint.getPosition();
  double q_target_ctrl = joint.getJointPositionCommand();
  MatrixXd L = joint.getL();
  VectorXd Kp = *joint.Kp_;
  VectorXd Kd = *joint.Kd_;
  int joint_index = joint.getJointIndex();

  double p_error = wrap_pos_neg_pi(q_ctrl - q_target_ctrl);
  // we use the joint_index column of the L matrix to calculate the result for this joint only
  VectorXd ld = L.col(joint_index) * (Kd[joint_index] * (p_error - p_error_last[joint_index])/period.seconds() + Kp[joint_index] * p_error);
  joint.setMotorCommand(ld);
  p_error_last[joint_index] = p_error;
  last_update[joint_index] = time;
}

void RobotHardware::print_vec(VectorXd vec, std::string name) {
  VectorXd vec_print;
  vec_print.resize(vec.size());
  for(size_t i=0; i<vec.size(); i++){
    if (vec[i] < 0.0001)
      vec_print[i] = 0.0;
    else
      vec_print[i] = vec[i];
  }
  std::stringstream ss1;
  ss1 << name << ": " << vec_print.transpose().format(fmt);
  RCUTILS_LOG_INFO_THROTTLE(RCUTILS_STEADY_TIME, 100, ss1.str().c_str());
  // RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), ss1.str().c_str());
}

void RobotHardware::publishViz() {
  if ((1.0 / (rclcpp::Clock().now() - last_visualization).seconds()) < 30) {
    { // tendon state publisher
      roboy_simulation_msgs::msg::Tendon msg;
      VectorXd l;
      l.resize(kinematics.number_of_cables);
      kinematics.getRobotCableFromJoints(l);
      for (size_t i = 0; i < kinematics.number_of_cables; i++) {
          msg.name.push_back(kinematics.cables[i].name);
          msg.force.push_back(kinematics.cable_forces[i]);
          msg.l.push_back(l[i]);
          msg.ld.push_back(Ld[0][i]); // TODO: only first endeffector Ld is send here
          msg.number_of_viapoints.push_back(kinematics.cables[i].viaPoints.size());
          for (shared_ptr<ViaPoint> vp : kinematics.cables[i].viaPoints) {
            geometry_msgs::msg::Vector3 vp_msg;
            // tf2::convert(vp->global_coordinates, vp_msg);
            vp_msg.x = vp->global_coordinates(0);
            vp_msg.y = vp->global_coordinates(1);
            vp_msg.z = vp->global_coordinates(2);
            msg.viapoints.push_back(vp_msg);
          }
      }
      tendon_state_pub->publish(msg);
    }
    { // robot state publisher
      static int seq = 0;
      vector<Matrix4d> robot_poses = kinematics.getRobotPosesFromJoints();
      for (size_t i = 0; i < kinematics.number_of_links; i++) {
        geometry_msgs::msg::PoseStamped msg;
        // msg.header.seq = seq++; // In ROS2, seq was removed.
        msg.header.stamp = rclcpp::Clock().now();
        msg.header.frame_id = kinematics.link_names[i];
        Isometry3d iso(robot_poses[i]);

        // tf::poseEigenToMsg(iso, msg.pose);
        Eigen::Quaterniond q = (Eigen::Quaterniond)iso.linear();
        msg.pose.position.x = iso.translation()[0];
        msg.pose.position.y = iso.translation()[1];
        msg.pose.position.z = iso.translation()[2];
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();
        if (msg.pose.orientation.w < 0) {
          msg.pose.orientation.x *= -1;
          msg.pose.orientation.y *= -1;
          msg.pose.orientation.z *= -1;
          msg.pose.orientation.w *= -1;
        }
        robot_state_pub->publish(msg);
      }
    }
    { // robot target publisher
      if((q_target-q_target_prev).norm()>0.001 || (qd_target-qd_target_prev).norm()>0.001 || first_update) { // only if target changed
        if(first_update)
          first_update = false;
        q_target_prev = q_target;
        qd_target_prev = qd_target;

        kinematics.setRobotState(q_target, qd_target);
        vector<Matrix4d> target_poses = kinematics.getRobotPosesFromJoints();
        // static int seq = 0;
        for (size_t i = 0; i < kinematics.number_of_links; i++) {
          geometry_msgs::msg::PoseStamped msg;
          // msg.header.seq = seq++; // In ROS2, seq was removed.
          msg.header.stamp = rclcpp::Clock().now();
          msg.header.frame_id = kinematics.link_names[i];
          Isometry3d iso(target_poses[i]);

          // tf::poseEigenToMsg(iso, msg.pose);
          Eigen::Quaterniond q = (Eigen::Quaterniond)iso.linear();
          msg.pose.position.x = iso.translation()[0];
          msg.pose.position.y = iso.translation()[1];
          msg.pose.position.z = iso.translation()[2];
          msg.pose.orientation.x = q.x();
          msg.pose.orientation.y = q.y();
          msg.pose.orientation.z = q.z();
          msg.pose.orientation.w = q.w();
          if (msg.pose.orientation.w < 0) {
            msg.pose.orientation.x *= -1;
            msg.pose.orientation.y *= -1;
            msg.pose.orientation.z *= -1;
            msg.pose.orientation.w *= -1;
          }

          robot_state_target_pub->publish(msg);
        }

        kinematics.getRobotCableFromJoints(l_target);
      }
    }
    { // joint state publisher
      sensor_msgs::msg::JointState cf_msg;
      roboy_simulation_msgs::msg::JointState msg;
      msg.names = kinematics.joint_names;
      // cf_msg.name = kinematics.joint_names; // TODO names missing???
      cf_msg.header.stamp = rclcpp::Clock().now();

      kinematics.setRobotState(q, qd);

      for (size_t i = 1; i < kinematics.number_of_links; i++) {
        Matrix4d pose = kinematics.getPoseFromJoint(i);
        Vector3d axis;
        axis << kinematics.joint_axis[i - 1][3], kinematics.joint_axis[i - 1][4], kinematics.joint_axis[i - 1][5];
        axis = pose.block(0, 0, 3, 3) * axis;
        // msg.origin.push_back(convertEigenToGeometry(pose.topRightCorner(3, 1)));
        geometry_msgs::msg::Vector3 geom_msg1;
        geom_msg1.x = pose.topRightCorner(3, 1)(0, 0);
        geom_msg1.y = pose.topRightCorner(3, 1)(1, 0);
        geom_msg1.z = pose.topRightCorner(3, 1)(2, 0);
        msg.origin.push_back(geom_msg1);
        // msg.axis.push_back(convertEigenToGeometry(axis));
        geometry_msgs::msg::Vector3 geom_msg2;
        geom_msg2.x = axis(0);
        geom_msg2.y = axis(1);
        geom_msg2.z = axis(2);
        msg.axis.push_back(geom_msg2);

        msg.torque.push_back(kinematics.torques[i - 1]);
        msg.q.push_back(q[i-1]);
        msg.qd.push_back(qd[i-1]);

        cf_msg.position.push_back(q[i-1]);
        cf_msg.velocity.push_back(qd[i-1]);
      }

      joint_state_pub->publish(msg);
      cardsflow_joint_states_pub->publish(cf_msg);
    }
  }
}


void RobotHardware::JointTarget(const sensor_msgs::msg::JointState::SharedPtr msg){ 
  int i = 0;
  if (msg->position.size() == msg->name.size())
  {
    for (string joint : msg->name) {
      int joint_index = kinematics.GetJointIdByName(joint);
      if (joint_index != -1) {
        if (msg->position[i] > kinematics.q_max(joint_index)) {
          q_target(joint_index)  = kinematics.q_max(joint_index);
        }
        else if (msg->position[i] < kinematics.q_min(joint_index)) {
          q_target(joint_index)  = kinematics.q_min(joint_index);
        }
        else {
          q_target(joint_index) = msg->position[i];
        }
        if (q_target(joint_index) < -0.001 || 0.001 < q_target(joint_index))
          RCLCPP_INFO(rclcpp::get_logger("RobotHardware"), "for %s set q_target(%d) = %f", joint.c_str(), joint_index, q_target(joint_index));
        // qd_target(joint_index) = msg->velocity[i];
      } else {
        RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 5000, "joint %s not found in model", joint.c_str());
      }
      i++;
    }
  }
}

void RobotHardware::JointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCUTILS_LOG_WARN_THROTTLE(RCUTILS_STEADY_TIME, 10000, "external joint states sub");
  int i = 0;
  for (string joint:msg->name) {
    if (std::count(kinematics.joint_names.begin(), kinematics.joint_names.end(), joint)) {
      int joint_index = kinematics.GetJointIdByName(joint);
      if (joint_index != -1) {
        q(joint_index) = msg->position[i];
        // qd(joint_index) = msg->velocity[i];
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "joint %s not found in model", joint.c_str());
      }
    }
    i++;
  }
}

void RobotHardware::controllerType(const roboy_simulation_msgs::msg::ControllerType::SharedPtr msg) {
  auto it = find(kinematics.joint_names.begin(), kinematics.joint_names.end(),msg->joint_name);
  if(it!=kinematics.joint_names.end()) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"%s changed controller to %s", msg->joint_name.c_str(),
                 (msg->type==CARDSflow::ControllerType::cable_length_controller?"cable_length_controller":
                  msg->type==CARDSflow::ControllerType::torque_position_controller?"torque_position_controller":
                  msg->type==CARDSflow::ControllerType::force_position_controller?"force_position_controller":"UNKNOWN"));
    controller_type[distance(kinematics.joint_names.begin(), it)] = msg->type;
  }
}

bool RobotHardware::FreezeService(std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res) {
  RCLCPP_INFO(rclcpp::get_logger("RobotHardware"),"FreezeService is called");
  for (size_t i = 1; i < kinematics.number_of_links; i++) {
    q_target = q;
  }
  res->message = "Robot stopped until the next joint target message";
  res->success = true;
  return true;
}

void RobotHardware::ZeroJoints(const roboy_control_msgs::msg::Strings::SharedPtr msg) {
  if (msg->names.empty()) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("RobotHardware"),"Setting all joint targets to zero");
    //for (int i = 1; i < number_of_links; i++) {
    q_target.setZero();
    //}
  }
  else {
    for (string joint: msg->names) {
      RCLCPP_INFO_STREAM(rclcpp::get_logger("RobotHardware"),"zero " << joint);
      int joint_index = kinematics.GetJointIdByName(joint);
      if (joint_index != iDynTree::JOINT_INVALID_INDEX) {
        q_target(joint_index) = 0;
      }
    };
  }
}
