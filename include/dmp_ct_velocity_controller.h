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

  void initROSCommunication();

  bool checkRobotSetup();

  bool loadDmpData(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                   std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  void initDmpObjects(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                      std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  void initCouplingObject (double &dt, std::vector<double> &gainA, std::vector<double> &gainB);

  void advanceCouplingTerm();

  void checkStoppingCondition();

  void commandRobot(const std::vector<double> &dq);

  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsedTime;

  // handle for robot hardware (not sure if safe?)
  hardware_interface::RobotHW* robotHardware;

  // handle for ROS node (communication, maybe not the best idea - performance?)
  ros::NodeHandle* nodeHandle;

  int dofs;
  // dmp class
  DiscreteDMP dmp;
  // reference dmp joint states without coupling term
  std::vector<std::vector<double>> refDmpTraj;
  // reference dmp joint velocity states without coupling term
  std::vector<std::vector<double>> refDmpVel;
  // iterator for reference step
  int iterateRef;
  // coupling term smoothing dmp
  DiscreteDMP couplingDmp;
  // time scaling factor: tau<1 -> slower execution
  double tau; 
  // timestep scaling factor between coupling term dmp and regular dmp (avoids oscillations)
  // i.e. the coupling term advances faster than the regular dmp
  int scaleCoupling;
  // time at which the coupling term interpolation reaches goal
  double timeCouplingFinal;
  std::vector<double> externalForce;
  // current coupling term per joint
  std::vector<double> couplingTerm;

  // current joint position of the dmp
  std::vector<double> qDmp;
  std::string robotIp;

  // callback function reacting to coupling term input
  void ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  ros::Subscriber subCoupling;

  // callback function reacting to coupling term input
  void smoothCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  ros::Subscriber smoothCoupling;

  // publisher for execution status flag
  ros::Publisher pubExec;
  // publisher for reward batches
  ros::Publisher pubBatch;
  // samplesBatch message
  common_msgs::SamplesBatch msgBatch;
  // current CouplingTerm message received
  common_msgs::CouplingTerm msgCoupling;
  // current temporary sample we add to the batch
  common_msgs::MDPSample tempMsg;

  bool flagPubEx  = false;
};

}  // namespace prcdmp_node
