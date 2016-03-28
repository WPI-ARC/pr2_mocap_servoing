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
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

#define _USE_MATH_DEFINES
#define PR2_ARM_JOINTS 7
#define TWIST_DOF 6

// Set the level of debugging verbosity
#define VERBOSE_DEBUGGING

// Set the "unit" maximum joint change/sec
#define MAXIMUM_JOINT_VELOCITY 1.0
// Set the control rate
#define CONTROL_RATE 5.0
// From the control rate, set the other core operating values
#define CONTROL_INTERVAL (1.0 / CONTROL_RATE)
#define EXECUTION_INTERVAL (2.0 * CONTROL_INTERVAL)
#define WATCHDOG_INTERVAL (3.0 * CONTROL_INTERVAL)
#define MAXIMUM_JOINT_CORRECTION (MAXIMUM_JOINT_VELOCITY * CONTROL_INTERVAL)

// Set the default PID gains
#define DEFAULT_KP 1.0
#define DEFAULT_KI 0.0
#define DEFAULT_KD 0.0

// Set damping for each of the arm joints
#define SHOULDER_PAN_DAMPING 0.5
#define SHOULDER_LIFT_DAMPING 0.5
#define UPPER_ARM_ROLL_DAMPING 0.75
#define ELBOW_FLEX_DAMPING 0.75
#define FOREARM_ROLL_DAMPING 1.0
#define WRIST_FLEX_DAMPING 1.0
#define WRIST_ROLL_DAMPING 1.0

// Set additional offsets for each of the arm joints
#define SHOULDER_PAN_OFFSET 0.0
#define SHOULDER_LIFT_OFFSET 0.0
#define UPPER_ARM_ROLL_OFFSET 0.0
#define ELBOW_FLEX_OFFSET 0.0
#define FOREARM_ROLL_OFFSET 0.0
#define WRIST_FLEX_OFFSET 0.0
#define WRIST_ROLL_OFFSET 0.0

#ifndef MOCAP_SERVOING_CONTROLLER_HPP
#define MOCAP_SERVOING_CONTROLLER_HPP

namespace pr2_mocap_servoing
{
    typedef Eigen::Affine3d Pose;
    typedef Eigen::Matrix<double, TWIST_DOF, 1> Twist;

    class MocapServoingController
    {
    protected:

        enum OPERATING_MODE {INTERNAL_POSE, EXTERNAL_POSE};
        OPERATING_MODE mode_;

        enum OPERATING_STATE {PAUSED, RUNNING};
        OPERATING_STATE state_;

        enum OPERATING_SIDE {LEFT, RIGHT};
        OPERATING_SIDE side_;

        std::vector<std::string> joint_names_;

        double max_joint_correction_;
        double execution_timestep_;

        double watchdog_timeout_;

        ros::NodeHandle nh_;

        robot_model::RobotModelPtr pr2_model_;
        robot_model::RobotStatePtr pr2_kinematic_state_;
        std::unique_ptr<robot_model::JointModelGroup> pr2_arm_group_;
        std::unique_ptr<const robot_model::LinkModel> pr2_arm_link_;
        std::unique_ptr<const robot_model::LinkModel> pr2_torso_link_;

        bool arm_pose_valid_;
        bool target_pose_valid_;
        bool arm_config_valid_;
        Pose current_arm_pose_;
        Pose current_target_pose_;
        std::vector<double> current_arm_config_;

        // Storage for PID control
        Twist pose_error_integral_;
        Twist last_pose_error_;
        double kp_;
        double ki_;
        double kd_;

        ros::Subscriber arm_pose_sub_;
        ros::Subscriber target_pose_sub_;
        ros::Subscriber arm_config_sub_;

        ros::ServiceServer abort_server_;

        std::unique_ptr<actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction>> arm_client_;

        ros::Timer arm_pose_watchdog_;
        ros::Timer target_pose_watchdog_;
        ros::Timer arm_config_watchdog_;

        inline void ArmPoseCB(geometry_msgs::PoseStamped arm_pose)
        {
            // First, check to make sure the frame is correct (someday, we'll use TF to make this more general)
            if (arm_pose.header.frame_id != std::string("/torso_lift_link") && arm_pose.header.frame_id != std::string("torso_lift_link"))
            {
                ROS_ERROR("Invalid frame for arm pose update - pose must be in /torso_lift_link frame");
            }
            // If the pose is safe, handle it
            else
            {
                // Convert to Eigen
                Eigen::Translation3d translation(arm_pose.pose.position.x, arm_pose.pose.position.y, arm_pose.pose.position.z);
                Eigen::Quaterniond rotation(arm_pose.pose.orientation.w, arm_pose.pose.orientation.x, arm_pose.pose.orientation.y, arm_pose.pose.orientation.z);
                Pose new_arm_pose = translation * rotation;
                // Set the pose
                current_arm_pose_ = new_arm_pose;
                // Set the status
                arm_pose_valid_ = true;
                // Check and set global status
                RefreshGlobalStatus();
                // Reset watchdog timer
                arm_pose_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmPoseWatchdogCB, this, true);
            }
        }

        inline void ArmPoseWatchdogCB(const ros::TimerEvent& e)
        {
            ROS_WARN("Arm pose hasn't been updated in %f seconds - pausing execution until a new pose update received", watchdog_timeout_);
            arm_pose_valid_ = false;
            state_ = PAUSED;
        }

        inline void TargetPoseCB(geometry_msgs::PoseStamped target_pose)
        {
            // First, check to make sure the frame is correct (someday, we'll use TF to make this more general)
            if (target_pose.header.frame_id != std::string("/torso_lift_link") && target_pose.header.frame_id != std::string("torso_lift_link"))
            {
                ROS_ERROR("Invalid frame for target pose update - pose must be in /torso_lift_link frame");
            }
            // Check if the provided pose is a special "cancel target" message
            else if (target_pose.pose.orientation.x == 0.0 && target_pose.pose.orientation.y == 0.0 && target_pose.pose.orientation.z == 0.0 && target_pose.pose.orientation.w == 0.0)
            {
                ROS_INFO("Cancelling pose target, switching to PAUSED mode");
                // Set the status
                target_pose_valid_ = false;
                // Check and set the global status
                RefreshGlobalStatus();
                // We don't reset the timer, instead we cancel it
                target_pose_watchdog_.stop();
            }
            // If the pose is safe, handle it
            else
            {
                // Convert to Eigen
                Eigen::Translation3d translation(target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
                Eigen::Quaterniond rotation(target_pose.pose.orientation.w, target_pose.pose.orientation.x, target_pose.pose.orientation.y, target_pose.pose.orientation.z);
                Pose new_target_pose = translation * rotation;
                // Set the pose
                current_target_pose_ = new_target_pose;
                // Set the status
                target_pose_valid_ = true;
                // Check and set global status
                RefreshGlobalStatus();
                // Reset watchdog timer
                target_pose_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::TargetPoseWatchdogCB, this, true);
            }
        }

        inline void TargetPoseWatchdogCB(const ros::TimerEvent& e)
        {
            ROS_WARN("Target pose hasn't been updated in %f seconds - continuing to current target", watchdog_timeout_);
        }

        inline void ArmConfigCB(pr2_controllers_msgs::JointTrajectoryControllerState arm_config)
        {
            // Extract joint positions in the right order
            if (arm_config.joint_names.size() != arm_config.actual.positions.size() || arm_config.joint_names.size() != PR2_ARM_JOINTS)
            {
                ROS_ERROR("Malformed configuration update - skipping update");
                return;
            }
            std::map<std::string, double> arm_configuration;
            for (size_t idx = 0; idx < arm_config.joint_names.size(); idx ++)
            {
                arm_configuration[arm_config.joint_names[idx]] = arm_config.actual.positions[idx];
            }
            //std::cout << "Got updated config: " << PrettyPrint(arm_configuration, true) << std::endl;
            // Set the config
            std::vector<double> new_arm_config(PR2_ARM_JOINTS);
            new_arm_config[0] = arm_configuration[joint_names_[0]];
            new_arm_config[1] = arm_configuration[joint_names_[1]];
            new_arm_config[2] = arm_configuration[joint_names_[2]];
            new_arm_config[3] = arm_configuration[joint_names_[3]];
            new_arm_config[4] = arm_configuration[joint_names_[4]];
            new_arm_config[5] = arm_configuration[joint_names_[5]];
            new_arm_config[6] = arm_configuration[joint_names_[6]];
            current_arm_config_ = new_arm_config;
            // If we're in INTERNAL_POSE mode, compute the arm pose
            if (mode_ == INTERNAL_POSE)
            {
                current_arm_pose_ = ComputeArmPose(current_arm_config_);
            }
            // Set the status
            arm_config_valid_ = true;
            // Check and set global status
            RefreshGlobalStatus();
            // Reset watchdog timer
            arm_config_watchdog_ = nh_.createTimer(ros::Duration(watchdog_timeout_), &MocapServoingController::ArmConfigWatchdogCB, this, true);
        }

        inline void ArmConfigWatchdogCB(const ros::TimerEvent& e)
        {
            ROS_WARN("Arm config hasn't been updated in %f seconds - pausing execution until a new config update received", watchdog_timeout_);
            arm_config_valid_ = false;
            state_ = PAUSED;
        }

        inline bool AbortCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
        {
            // Cancel the current pose target
            ROS_INFO("Cancelling pose target, switching to PAUSED mode");
            // Set the status
            target_pose_valid_ = false;
            // Check and set the global status
            RefreshGlobalStatus();
            // We don't reset the timer, instead we cancel it
            target_pose_watchdog_.stop();
            return true;
        }

        inline Pose ComputeArmPose(std::vector<double>& current_configuration)
        {
            // Update joint values
            pr2_kinematic_state_->setJointGroupPositions(pr2_arm_group_.get(), current_configuration);
            // Update the joint transforms
            pr2_kinematic_state_->update(true);
            // Get the transform from base to torso
            Pose current_base_to_torso_pose = pr2_kinematic_state_->getGlobalLinkTransform(pr2_torso_link_.get());
            // Get the transform from base to wrist
            Pose current_base_to_wrist_pose = pr2_kinematic_state_->getGlobalLinkTransform(pr2_arm_link_.get());
            // Get the arm pose
            Pose current_arm_pose = current_base_to_torso_pose.inverse() * current_base_to_wrist_pose;
            return current_arm_pose;
        }

        inline Eigen::MatrixXd ComputeJacobian(std::vector<double>& current_configuration)
        {
            // Update joint values
            pr2_kinematic_state_->setJointGroupPositions(pr2_arm_group_.get(), current_configuration);
            // Update the joint transforms
            pr2_kinematic_state_->update(true);
            // Compute the Jacobian
            Eigen::MatrixXd current_jacobian = pr2_kinematic_state_->getJacobian(pr2_arm_group_.get());
            return current_jacobian;
        }

        inline Twist ComputePoseError(Pose& arm_pose, Pose& target_pose)
        {
            std::cout << "Current arm pose:\n" << arm_pose.translation() << std::endl;
            std::cout << "Current target pose:\n" << target_pose.translation() << std::endl;
            Twist pose_error;
            pose_error.head<3>() = arm_pose.translation() - target_pose.translation();
            pose_error.tail<3>() = 0.5 * (target_pose.linear().col(0).cross(arm_pose.linear().col(0)) + target_pose.linear().col(1).cross(arm_pose.linear().col(1)) + target_pose.linear().col(2).cross(arm_pose.linear().col(2)));
            pose_error = -1.0 * pose_error;
            std::cout << "Computed pose error:\n" << pose_error << std::endl;
            return pose_error;
        }

        inline void RefreshGlobalStatus()
        {
            if (mode_ == INTERNAL_POSE)
            {
                if (target_pose_valid_ && arm_config_valid_)
                {
                    state_ = RUNNING;
                }
                else
                {
                    state_ = PAUSED;
                }
            }
            else
            {
                if (arm_pose_valid_ && target_pose_valid_ && arm_config_valid_)
                {
                    state_ = RUNNING;
                }
                else
                {
                    state_ = PAUSED;
                }
            }
        }

        std::vector<double> ComputeNextStep(Pose& current_arm_pose, Pose& current_target_pose, std::vector<double>& current_configuration);

        void CommandToTarget(std::vector<double>& current_config, std::vector<double>& target_config);

    public:

        MocapServoingController(ros::NodeHandle& nh, std::string group_name, std::string arm_pose_topic, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd);

        MocapServoingController(ros::NodeHandle &nh, std::string group_name, std::string target_pose_topic, std::string arm_config_topic, std::string arm_command_action, std::string abort_service, double kp, double ki, double kd);

        void Loop();
    };
}

#endif // MOCAP_SERVOING_CONTROLLER_HPP
