#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iostream>
#include <random>
#include <Eigen/Geometry>
#include <time.h>
#include <chrono>
#include <ros/ros.h>
#include <urdf_model/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <pr2_mocap_servoing/mocap_servoing_controller.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pr2_right_arm_mocap_servoing_controller");
    ROS_INFO("Starting pr2_right_arm_mocap_servoing_controller (w/o mocap)...");
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    std::string target_pose_topic;
    std::string arm_config_topic;
    std::string arm_command_action;
    //double execution_timestep = 0.1;
    double kp = DEFAULT_KP;
    double ki = DEFAULT_KI;
    double kd = DEFAULT_KD;
    nhp.param(std::string("target_pose_topic"), target_pose_topic, std::string("/r_arm_pose_controller/target"));
    nhp.param(std::string("arm_config_topic"), arm_config_topic, std::string("/r_arm_controller/state"));
    nhp.param(std::string("arm_command_action"), arm_command_action, std::string("/r_arm_controller/joint_trajectory_action"));
    //nhp.param(std::string("execution_timestep"), execution_timestep, 0.1);
    nhp.param(std::string("kp"), kp, DEFAULT_KP);
    nhp.param(std::string("ki"), ki, DEFAULT_KI);
    nhp.param(std::string("kd"), kd, DEFAULT_KD);
    ROS_INFO("Running in INTERNAL_POSE mode");
    pr2_mocap_servoing::MocapServoingController controller(nh, std::string("right_arm"), target_pose_topic, arm_config_topic, arm_command_action, kp, ki, kd);
    ROS_INFO("...startup complete");
    controller.Loop();
    return 0;
}
