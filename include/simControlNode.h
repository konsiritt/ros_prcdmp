// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>
#include <random>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <boost/array.hpp>

#include "control_msgs/FollowJointTrajectoryAction.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64MultiArray.h"
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

namespace prcdmp_node {

class simControlNode {

public:
 simControlNode(ros::NodeHandle& node_handle);

 bool init(ros::NodeHandle& node_handle);
 bool update(const ros::Time&, const ros::Duration& period);
 void starting(const ros::Time&);
 void stopping(const ros::Time&);

 bool getMdpReady() {return mdpReady;};

private:
 void initROSCommunication();

 bool loadDmpData(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                  std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

 void initDmpObjects(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                    std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

 bool checkStoppingCondition();

 void commandRobot(const std::vector<double> &dq);

 void ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
 void ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
 void frankaStateCallback(const sensor_msgs::JointState& msg);//(const franka_msgs::FrankaState::ConstPtr& msg);
 void mdpReadyCallback(const std_msgs::Bool::ConstPtr& msg);

 bool isValidVelocity(std::vector<double> velocitiesToApply);

 void setupSampling();
 std::vector<double> getRandomVectorOffset();
 std::vector<double> addVectors(const std::vector<double>& element1, const std::vector<double>& element2);
 void sampleDmpInit();
 void sampleGoalQ();
 std::vector<double> sampleVector (const std::vector<double> inputVector);

 void computeGoalOffset();

 void saveDmpData();

 ros::Duration elapsedTime;


 // ROS communication
 ros::NodeHandle* nodeHandle; // handle for ROS node
 ros::Subscriber subCoupling;
 ros::Subscriber subCouplingSmoothed;
 ros::Subscriber subFrankaStates;
 ros::Subscriber subMdpReady;
 ros::Publisher pubExec;
 ros::Publisher pubError;
 ros::Publisher pubBatch;
 ros::Publisher pubGoal;
 ros::Publisher pubInit;
 common_msgs::SamplesBatch ctBatch;
 common_msgs::MDPSample ctSample;
 ros::ServiceClient collisionClient;

 // ROS action command for robot trajectory signals
 actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajClient;
 control_msgs::FollowJointTrajectoryGoal actionGoal;

 // states concerning communication and robot
 uint8_t currentRobotMode;
 bool flagPubEx  = false;
 bool flagPubErr = false;
 bool logging = false;
 bool mdpReady = false;

 // dmp specific members
 int dofs;
 DiscreteDMP dmp; // dmp class
 double tau; // time scaling factor: tau<1 -> slower execution
 std::vector<double> externalForce;

 // trajectory specific members
 std::vector <double> dmpQ0; // initial joint position in the dmp
 boost::array<double,7> dmpGoal; // goal joint position in the dmp
 boost::array<double,7> robotQ; // current joint position of the robot
 std::string robotIp;
 std::vector<std::vector<double>> refQ; // reference trajectory for rollout without coupling term
 std::vector<std::vector<double>> saveDmpQ; // member that records qs to save to file
 std::vector<std::vector<double>> saveRobotQ; // member that records robot qs to save to file
 std::vector<double> saveTime;
 int refIter=-1; // iterator for reference trajectory
 bool firstCB = true;

 double virtWallZ_EE = 0.10;// virtual wall for end effector height in 0_T
 double virtWallZ_F = 0.15;// virtual wall for end effector height in 0_T
 double virtWallZ_J7 = 0.15;// virtual wall for end effector height in 0_T
 double virtWallZ_J6 = 0.15;// virtual wall for end effector height in 0_T

 //include random generator to sample initial position
 std::default_random_engine generator;
 std::normal_distribution<double> distribution;
 int seed = 0; //= std::chrono::system_clock::now().time_since_epoch().count();
 double meanOffset = 0.0;
 double stdOffset = 0.05;

 std::string datasetPath;
};

}  // namespace prcdmp_node
