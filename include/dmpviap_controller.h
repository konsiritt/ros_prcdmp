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
#include <actionlib/client/simple_action_client.h>

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
#include <franka_control/ErrorRecoveryAction.h>

namespace prcdmp_node {

class DmpViapController : public controller_interface::MultiInterfaceController<
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

  bool saveRobotState();// updates robotQ to the current robot state

  void initDmpObjects(double &dt, std::vector<double> &y0v, std::vector<double> &goalv,
                      std::vector<double> &gainA, std::vector<double> &gainB);

  bool checkRobotInit();

  void checkRobotState();

  void checkStoppingCondition();

  void commandRobot(const std::vector<double> &dq);

  bool errorRecovery();

  void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg);

  // handles for robot
  hardware_interface::RobotHW* robotHardware;
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

  ros::Duration elapsedTime;

  // ROS communication
  ros::NodeHandle* nodeHandle;
  ros::Publisher pub;
  ros::Subscriber subFrankaStates;

  // dmp specific members
  int dofs;
  DiscreteDMP dmpInitialize; // dmp class
  double timeSpan = 3.5;// for initializing the timespan can be shorter
  double tau; // time scaling factor: tau<1 -> slower execution
  std::vector<double> externalForce;

  // trajectory specific members
  std::vector<double> dmpQ0; // initial joint position in the dmp
  std::array<double,7> robotQ; // current joint position of the robot
  // safe via point for recovery after passing the obstacle
  std::vector<double> viaPointQ = {0.512606,-0.692921,-0.208256,-1.87101,1.2428,1.86703,-0.287594};//{0.57189,0.293397,0.392036,-1.20435,1.32246,2.51705,-0.55729};//

  std::string robotIp;

  // states concerning robot and communication
  uint8_t currentRobotMode;
  bool notInitializedDMP = false; // flag whether or not moving to start is necessary
  bool executingDMP = false;// flag wheter or not the target dmp is being executed
  bool flagPubEx  = false;  
};

}  // namespace prcdmp_node
