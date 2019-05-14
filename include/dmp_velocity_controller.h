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
#include <actionlib/client/simple_action_client.h>
#include <franka_control/ErrorRecoveryAction.h>

#include "std_msgs/Bool.h"
#include "common_msgs/CouplingTerm.h"
#include "common_msgs/MDPSample.h"
#include "common_msgs/SamplesBatch.h"
#include "franka_msgs/FrankaState.h"

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

class DmpVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  void initROSCommunication();

  bool checkRobotSetup();

  bool loadDmpData(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                   std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  void initDmpObjects(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                      std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  bool checkRobotInit();

  void checkRobotState();

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
  // time scaling factor: tau<1 -> slower execution
  double tau; 
  std::vector<double> externalForce;

  // initial joint position in the dmp
  std::array<double,7> q0;
  // current joint position of the robot
  std::array<double,7> qInit;
  std::string robotIp;
  std::vector<std::vector<double>> refQ;
  int refIter=-1;
  bool firstCB = true;


  // dummy (for now) callback function reacting to boolean input
  void ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  void ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg);
  ros::Subscriber subCoupling;
  ros::Subscriber subCouplingSmoothed;
  ros::Subscriber subFrankaStates;

  uint8_t currentRobotMode;

  bool errorRecovery();

  common_msgs::SamplesBatch ctBatch;
  common_msgs::MDPSample ctSample;

  // publisher for execution status flag
  ros::Publisher pubExec;
  ros::Publisher pubBatch;

  bool flagPubEx  = false;
};

}  // namespace prcdmp_node
