// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>
#include <random>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <boost/array.hpp>

#include "std_msgs/Bool.h"
#include "common_msgs/CouplingTerm.h"
#include "common_msgs/MDPSample.h"
#include "common_msgs/SamplesBatch.h"
#include "franka_msgs/FrankaState.h"

#include "UTILS/Config.h"
#include "UTILS/trajectoryUtils.h"
#include "DMP/DMP.hpp"
#include "DMP/DiscreteDMP.hpp"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>
#include <franka/exception.h>
#include <franka_control/ErrorRecoveryAction.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_control/services.h>

namespace prcdmp_node {

class DmpVelocityController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaModelInterface,
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
  // sets qInit to the current robot state
  bool getRobotState();

  void checkStoppingCondition();

  void commandRobot(const std::vector<double> &dq);

  bool errorRecovery();

  void ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  void ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
  void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg);

  bool isValidVelocity(std::vector<double> velocitiesToApply);

  void setupSampling();
  std::vector<double> getRandomVectorOffset();
  std::vector<double> addVectors(const std::vector<double>& element1, const std::vector<double>& element2);
  void sampleGoalQ();

  void computeGoalOffset();

  void saveDmpData();

  ros::Duration elapsedTime;

  // robot handles
  hardware_interface::RobotHW* robotHardware;
  franka_hw::FrankaModelInterface* modelInterface;
  std::unique_ptr<franka_hw::FrankaModelHandle> modelHandle;
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;

  // ROS communication
  ros::NodeHandle* nodeHandle; // handle for ROS node
  ros::Subscriber subCoupling;
  ros::Subscriber subCouplingSmoothed;
  ros::Subscriber subFrankaStates;
  ros::Publisher pubExec;
  ros::Publisher pubError;
  ros::Publisher pubBatch;
  common_msgs::SamplesBatch ctBatch;
  common_msgs::MDPSample ctSample;
  ros::ServiceClient collisionClient;

  // states concerning communication and robot
  uint8_t currentRobotMode;
  bool flagPubEx  = false;
  bool flagPubErr = false;
  bool logging = false;

  // dmp specific members
  int dofs;
  DiscreteDMP dmp; // dmp class
  double tau; // time scaling factor: tau<1 -> slower execution
  std::vector<double> externalForce;

  // trajectory specific members
  std::array<double,7> dmpQ0; // initial joint position in the dmp
  boost::array<double,7> dmpGoal; // goal joint position in the dmp
  std::array<double,7> qInit; // current joint position of the robot
  std::string robotIp;
  std::vector<std::vector<double>> refQ; // reference trajectory for rollout without coupling term
  std::vector<std::vector<double>> saveQ; // member that records qs to save to file
  int refIter=-1; // iterator for reference trajectory
  bool firstCB = true;

  double virtWallZ_EE = 0.10;// virtual wall for end effector height in 0_T
  double virtWallZ_F = 0.15;// virtual wall for end effector height in 0_T
  double virtWallZ_J7 = 0.15;// virtual wall for end effector height in 0_T
  double virtWallZ_J6 = 0.15;// virtual wall for end effector height in 0_T

  //include random generator to sample initial position
  std::default_random_engine generator;
  std::normal_distribution<double> distribution;
  double meanOffset = 0.0;
  double stdOffset = 0.05;

  std::string datasetPath;
};

}  // namespace prcdmp_node
