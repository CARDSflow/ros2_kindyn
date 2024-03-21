#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("send_cmd");
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher = 
        node->create_publisher<sensor_msgs::msg::JointState>("/roboy/pinky/control/joint_targets", 10);
    
    auto joint_state_msg = sensor_msgs::msg::JointState();
    std::vector<std::string> names = {"elbow_left_axis0", "elbow_left_axis1",
                                      "elbow_right_axis0", "elbow_right_axis1",
                                      "head_axis0", "head_axis1", "head_axis2",
                                      "shoulder_left_axis0", "shoulder_left_axis1", "shoulder_left_axis2",
                                      "shoulder_right_axis0", "shoulder_right_axis1", "shoulder_right_axis2"};
                                      
    // hw_commands_[i].getJointPositionCommand() returns 0.0 for "shoulder_left_axis1" (8), "shoulder_left_axis2 (9),"shoulder_right_axis1" (11), "shoulder_right_axis2" (12)

    // head, head, shoulder_left, shoulder_left, l rs rs rs r l l 
    std::vector<double> min_limit = {0.0, 0.0,  0.0, 0.0, -0.6, -0.56, -0.6, -1.3, -1.2, -0.3, -1.0, -1.2, -0.3};
    std::vector<double> max_limit = {0.9, 0.73, 0.9, 0.73, 0.6,  0.56,  0.6,  0.3,  1.0,  0.3,  0.3,  0.0,  0.3};
    std::vector<double> neutral   = {0.0, 0.0,  0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos0 = {0.9, 0.0,  0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos1 = {0.0, 0.73, 0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos2 = {0.0, 0.0,  0.9, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos3 = {0.0, 0.0,  0.0, 0.73, 0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos4 = {0.0, 0.0,  0.0, 0.0,  0.6,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos5 = {0.0, 0.0,  0.0, 0.0,  0.0, 0.56,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos6 = {0.0, 0.0,  0.0, 0.0,  0.0,  0.0,   0.6,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos7 = {0.0, 0.0,  0.0, 0.0,  0.0,  0.0,   0.0,  -1.3,  0.0,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos8 = {0.0, 0.0,  0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  -1.2,  0.0,  0.0,  0.0,  0.0};
    std::vector<double> pos9 = {0.0, 0.0,  0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.3,  0.0,  0.0,  0.0};
    std::vector<double> pos10 = {0.0, 0.0, 0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  -1.0,  0.0,  0.0};
    std::vector<double> pos11 = {0.0, 0.0, 0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  -1.2,  0.0};
    std::vector<double> pos12 = {0.0, 0.0, 0.0, 0.0,  0.0,  0.0,   0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.3};

    std::vector<std::vector<double>> positions = {pos0, pos1, pos2, pos3, pos4, pos5, pos6, pos7, pos8, pos9, pos10, pos11, pos12};

    joint_state_msg.name = names;
    rclcpp::Rate rate_slow(0.5); // 0.5 Hz = 2 sec
    rclcpp::Rate rate_fast(2); // 2 Hz = 0.5 sec
    while (rclcpp::ok())
    {
        for (size_t i = 0; i < positions.size(); i++) {
            joint_state_msg.position = positions[i];
            publisher->publish(joint_state_msg);
            rate_slow.sleep();

            joint_state_msg.position = neutral;
            publisher->publish(joint_state_msg);
            rate_fast.sleep();
        }
    }

  return 0;
}