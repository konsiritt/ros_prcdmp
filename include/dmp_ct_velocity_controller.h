// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include "std_msgs/Bool.h"
#include "common_msgs/CouplingTerm.h"
#include "common_msgs/MDPSample.h"
#include "common_msgs/SamplesBatch.h"

#include "UTILS/Config.h"
#include <string>
#include "DMP/DMP.hpp"
#include "DMP/DiscreteDMP.hpp"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>

namespace prcdmp_node {

class DmpCtVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  // add the message information to the reward batch
  void addCurrMessage();

  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  // handle for robot hardware (not sure if safe?)
  hardware_interface::RobotHW* robotHardware;

  // handle for ROS node (communication, maybe not the best idea - performance?)
  ros::NodeHandle* nodeHandle;

  // dmp class
  DiscreteDMP dmp;
  // reference dmp without coupling term
  DiscreteDMP refDmp;
  // reference dmp joint states without coupling term
  std::vector<std::vector<double>> refDmpTraj;
  // iterator for reference step
  int iterateRef;
  // coupling term smoothing dmp
  DiscreteDMP couplingDmp;
  // time scaling factor: tau<1 -> slower execution
  double tau; 
  std::vector<double> externalForce;
  // current coupling term per joint
  std::vector<double> couplingTerm;

  // initial joint position in the dmp
  std::array<double,7> q0;
  // current joint position of the dmp
  std::vector<double> qDmp;
  // initial joint position of the robot
  std::array<double,7> qInit;
  std::string robotIp;

  // flag whether or not moving to start is necessary
  bool notInitializedDMP = false;
  // flag wheter or not the target dmp is being executed
  bool executingDMP = false;

  // dummy (for now) callback function reacting to boolean input
  void callback(const common_msgs::CouplingTerm::ConstPtr& msg);
  ros::Subscriber subCoupling;

  // publisher for execution status flag
  ros::Publisher pubExec;

  // publisher for reward batches
  ros::Publisher pubBatch;
  // samplesBatch message
  common_msgs::SamplesBatch msgBatch;
  // current CouplingTerm message received
  common_msgs::CouplingTerm msgCoupling;

  bool tempPublished  = false;
};

}  // namespace prcdmp_node
