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
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <pr2_mocap_servoing/mocap_servoing_controller.hpp>

using namespace pr2_mocap_servoing;

MocapServoingController::MocapServoingController(ros::NodeHandle& nh, std::string group_name, std::string arm_pose_topic, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd) : nh_(nh)
{
    // Set mode
    mode_ = EXTERNAL_POSE;
    // Set up an internal robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    pr2_model_ = robot_model_loader.getModel();
    pr2_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(pr2_model_));
    pr2_kinematic_state_->setToDefaultValues();
    pr2_kinematic_state_->update();
    if (group_name == std::string("left_arm"))
    {
        side_ = LEFT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("l_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "l_shoulder_pan_joint";
        joint_names_[1] = "l_shoulder_lift_joint";
        joint_names_[2] = "l_upper_arm_roll_joint";
        joint_names_[3] = "l_elbow_flex_joint";
        joint_names_[4] = "l_forearm_roll_joint";
        joint_names_[5] = "l_wrist_flex_joint";
        joint_names_[6] = "l_wrist_roll_joint";
    }
    else if (group_name == std::string("right_arm"))
    {
        side_ = RIGHT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("r_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "r_shoulder_pan_joint";
        joint_names_[1] = "r_shoulder_lift_joint";
        joint_names_[2] = "r_upper_arm_roll_joint";
        joint_names_[3] = "r_elbow_flex_joint";
        joint_names_[4] = "r_forearm_roll_joint";
        joint_names_[5] = "r_wrist_flex_joint";
        joint_names_[6] = "r_wrist_roll_joint";
    }
    else
    {
        throw std::invalid_argument("Invalid group name");
    }
    // Setup topics
    arm_pose_sub_ = nh_.subscribe(arm_pose_topic, 1, &MocapServoingController::ArmPoseCB, this);
    target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &MocapServoingController::TargetPoseCB, this);
    arm_config_sub_ = nh_.subscribe(arm_config_topic, 1, &MocapServoingController::ArmConfigCB, this);
    // Setup abort service
    abort_server_ = nh_.advertiseService(abort_service, &MocapServoingController::AbortCB, this);
    // Setup trajectory controller interface
    arm_client_ = std::unique_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>>(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(arm_command_action, true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();
    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    // Set max size for qdot
    max_joint_correction_ = MAXIMUM_JOINT_CORRECTION;
    // Set execution timestep
    execution_timestep_ = EXECUTION_INTERVAL;
    // Set timeout
    watchdog_timeout_ = WATCHDOG_INTERVAL;
    // Initialize the control variables to safe values
    arm_pose_valid_ = false;
    arm_config_valid_ = false;
    target_pose_valid_ = false;
    // Initialize the PID values to zero
    pose_error_integral_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    last_pose_error_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Start in PAUSED mode
    state_ = PAUSED;
}

MocapServoingController::MocapServoingController(ros::NodeHandle &nh, std::string group_name, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd) : nh_(nh)
{
    // Set mode
    mode_ = INTERNAL_POSE;
    // Set up an internal robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    pr2_model_ = robot_model_loader.getModel();
    pr2_kinematic_state_ = robot_model::RobotStatePtr(new robot_state::RobotState(pr2_model_));
    pr2_kinematic_state_->setToDefaultValues();
    pr2_kinematic_state_->update();
    if (group_name == std::string("left_arm"))
    {
        side_ = LEFT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("l_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "l_shoulder_pan_joint";
        joint_names_[1] = "l_shoulder_lift_joint";
        joint_names_[2] = "l_upper_arm_roll_joint";
        joint_names_[3] = "l_elbow_flex_joint";
        joint_names_[4] = "l_forearm_roll_joint";
        joint_names_[5] = "l_wrist_flex_joint";
        joint_names_[6] = "l_wrist_roll_joint";
    }
    else if (group_name == std::string("right_arm"))
    {
        side_ = RIGHT;
        pr2_arm_group_ = std::unique_ptr<robot_model::JointModelGroup>(pr2_model_->getJointModelGroup(group_name));
        pr2_arm_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("r_wrist_roll_link")));
        pr2_torso_link_ = std::unique_ptr<const robot_model::LinkModel>(pr2_kinematic_state_->getLinkModel(std::string("torso_lift_link")));
        // Set the joint names
        joint_names_.resize(PR2_ARM_JOINTS);
        joint_names_[0] = "r_shoulder_pan_joint";
        joint_names_[1] = "r_shoulder_lift_joint";
        joint_names_[2] = "r_upper_arm_roll_joint";
        joint_names_[3] = "r_elbow_flex_joint";
        joint_names_[4] = "r_forearm_roll_joint";
        joint_names_[5] = "r_wrist_flex_joint";
        joint_names_[6] = "r_wrist_roll_joint";
    }
    else
    {
        throw std::invalid_argument("Invalid group name");
    }
    // Setup topics
    target_pose_sub_ = nh_.subscribe(target_pose_topic, 1, &MocapServoingController::TargetPoseCB, this);
    arm_config_sub_ = nh_.subscribe(arm_config_topic, 1, &MocapServoingController::ArmConfigCB, this);
    // Setup abort service
    abort_server_ = nh_.advertiseService(abort_service, &MocapServoingController::AbortCB, this);
    // Setup trajectory controller interface
    arm_client_ = std::unique_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>>(new actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>(arm_command_action, true));
    ROS_INFO("Waiting for arm controllers to come up...");
    arm_client_->waitForServer();
    // Set gains
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    // Set max size for qdot
    max_joint_correction_ = MAXIMUM_JOINT_CORRECTION;
    // Set execution timestep
    execution_timestep_ = EXECUTION_INTERVAL;
    // Set timeout
    watchdog_timeout_ = WATCHDOG_INTERVAL;
    // Initialize the control variables to safe values
    arm_pose_valid_ = false;
    arm_config_valid_ = false;
    target_pose_valid_ = false;
    // Initialize the PID values to zero
    pose_error_integral_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    last_pose_error_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    // Start in PAUSED mode
    state_ = PAUSED;
}

std::vector<double> MocapServoingController::ComputeNextStep(Pose& current_arm_pose, Pose& current_target_pose, std::vector<double>& current_configuration)
{
    // Get the current jacobian
    Eigen::MatrixXd current_jacobian = ComputeJacobian(current_configuration);
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current Jacobian: " << current_jacobian << std::endl;
#endif
    // Compute the pose error in our 'world frame'
    Twist pose_error = ComputePoseError(current_arm_pose, current_target_pose);
    // Compute the integral of pose error & update the stored value
    pose_error_integral_ = pose_error_integral_ + (pose_error * CONTROL_INTERVAL);
    // Compute the derivative of pose error
    Twist pose_error_derivative = (pose_error - last_pose_error_) / CONTROL_INTERVAL;
    // Update the stored pose error
    last_pose_error_ = pose_error;
    // Convert pose errors into cartesian velocity
    Twist pose_correction = (pose_error * kp_) + (pose_error_integral_ * ki_) + (pose_error_derivative * kd_);
    // Use the Jacobian pseudoinverse
    Eigen::VectorXd joint_correction = EigenHelpers::Pinv(current_jacobian, EigenHelpers::SuggestedRcond()) * pose_correction;
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current raw joint correction: " << joint_correction << std::endl;
#endif
    // Bound qdot to max magnitude of 0.05
    double joint_correction_magnitude = joint_correction.norm();
    if (joint_correction_magnitude > max_joint_correction_)
    {
        joint_correction = (joint_correction / joint_correction_magnitude) * max_joint_correction_;
    }
#ifdef VERBOSE_DEBUGGING
    std::cout << "Current limited joint correction: " << joint_correction << std::endl;
#endif
    // Combine the joint correction with the current configuration to form the target configuration
    std::vector<double> target_configuration(PR2_ARM_JOINTS);
    target_configuration[0] = current_configuration[0] + (joint_correction[0] * SHOULDER_PAN_DAMPING) + SHOULDER_PAN_OFFSET;
    target_configuration[1] = current_configuration[1] + (joint_correction[1] * SHOULDER_LIFT_DAMPING) + SHOULDER_LIFT_OFFSET;
    target_configuration[2] = current_configuration[2] + (joint_correction[2] * UPPER_ARM_ROLL_DAMPING) + UPPER_ARM_ROLL_OFFSET;
    target_configuration[3] = current_configuration[3] + (joint_correction[3] * ELBOW_FLEX_DAMPING) + ELBOW_FLEX_OFFSET;
    target_configuration[4] = current_configuration[4] + (joint_correction[4] * FOREARM_ROLL_DAMPING) + FOREARM_ROLL_OFFSET;
    target_configuration[5] = current_configuration[5] + (joint_correction[5] * WRIST_FLEX_DAMPING) + WRIST_FLEX_OFFSET;
    target_configuration[6] = current_configuration[6] + (joint_correction[6] * WRIST_ROLL_DAMPING) + WRIST_ROLL_OFFSET;
    std::cout << "Current configuration: " << PrettyPrint::PrettyPrint(current_configuration, true) << std::endl;
    std::cout << "New target configuration: " << PrettyPrint::PrettyPrint(target_configuration, true) << std::endl;
    return target_configuration;
}

void MocapServoingController::Loop()
{
    ros::Rate spin_rate(CONTROL_RATE);
    while (ros::ok())
    {
        // Do the next step
        if (state_ == RUNNING)
        {
            // Compute the next step
            std::vector<double> target_config = ComputeNextStep(current_arm_pose_, current_target_pose_, current_arm_config_);
            // Command the robot
            CommandToTarget(current_arm_config_, target_config);
        }
        // Process callbacks
        ros::spinOnce();
        // Spin
        spin_rate.sleep();
    }
}

void MocapServoingController::CommandToTarget(std::vector<double>& current_config, std::vector<double>& target_config)
{
    pr2_controllers_msgs::JointTrajectoryGoal command;
    // Populate command
    command.trajectory.joint_names = joint_names_;
    command.trajectory.header.stamp = ros::Time::now();
    // Populate target point
    trajectory_msgs::JointTrajectoryPoint start_point;
    start_point.positions = current_config;
    start_point.velocities.resize(start_point.positions.size(), 0.0);
    start_point.time_from_start = ros::Duration(0.0);
    // Populate target point
    trajectory_msgs::JointTrajectoryPoint target_point;
    target_point.positions = target_config;
    target_point.velocities.resize(target_point.positions.size(), 0.0);
    // Set the execution time
    target_point.time_from_start = ros::Duration(execution_timestep_);
    // Add point
    command.trajectory.points.push_back(target_point);
    // Command the arm
    arm_client_->sendGoal(command);
}
