// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <dmp_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

/*
  // load config data from files
  std::string datasetPath = "set1"; //TODO: get via command line input or so?!

  // handles config file access
  Config config(datasetPath);

  //fill data from json to variables
  int dofs = config.getDmpJson()["dofs"].asInt();
  int nBFs = config.getDmpJson()["n_basis"].asInt();
  double dt = config.getDmpJson()["dt"].asDouble();
  double timeSpan = config.getDmpJson()["timespan"].asDouble();
  double tau = 1.0/timeSpan;

  // initialize arrays from config file
  std::array<double,7> q0;
  std::array<double,7> goal;
  std::vector<double> gainA, gainB;
  moveJsonArrayToVec(config.getDmpJson()["q0"], q0);
  moveJsonArrayToVec(config.getDmpJson()["goal"], goal);
  moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
  moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);

  //fill data from json to variables
  std::string robotIp = config.getDataJson()["robot_ip"].asString();
  std::cout<<"Robot ip : "<<robotIp<<std::endl;
  int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
  config.fillTrajectoryPath(episodeNr);

  Interface roboInter;
  std::vector<double> externalForce;
  std::vector<std::vector<double>> w ;
  if (episodeNr ==0) {
      UTILS::loadWeights(config.getInitialWPath(),w);
  }
  else {
      UTILS::loadWeights(config.getwPath(),w);
  }

  // convert arrays to vectors
  std::vector<double> y0v(q0.begin(), q0.end());
  std::vector<double> goalv(goal.begin(), goal.end());

  // initialize dmp object
  DiscreteDMP dmp(dofs, nBFs, dt, y0v, goalv, w, gainA, gainB);
*/

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  ros::Duration time_max(8.0);
  double omega_max = 0.1;
  double cycle = std::floor(
      std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
                         time_max.toSec()));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  for (auto joint_handle : velocity_joint_handles_) {
    joint_handle.setCommand(omega);
  }
}

void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
