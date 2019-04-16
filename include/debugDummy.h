#ifndef DEBUGDUMMY_H
#define DEBUGDUMMY_H

#include "ros/ros.h"
#include <ros/time.h>
#include <ros/package.h>
#include "std_msgs/Bool.h"
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>

#include "UTILS/Config.h"
#include <string>
#include "DMP/DMP.hpp"
#include "DMP/DiscreteDMP.hpp"
#include <jsoncpp/json/value.h>
#include <jsoncpp/json/json.h>

#include "common_msgs/CouplingTerm.h"
#include "common_msgs/MDPSample.h"
#include "common_msgs/SamplesBatch.h"



class debugDummy
{

public:
  debugDummy(ros::NodeHandle &node);

  ~debugDummy();

  ros::NodeHandle* nodeHandle;

  void update(const ros::Time&, const ros::Duration& period);

  void checkStoppingCondition();

private:
  void initializedCallback(const std_msgs::Bool::ConstPtr& msg);
  void executedCallback(const std_msgs::Bool::ConstPtr& msg);

  bool loadDmpData(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                   std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  void initDmpObjects(int &nBFs, double &dt,std::vector<double> &y0v, std::vector<double> &goalv,
                      std::vector<std::vector<double>> &w, std::vector<double> &gainA, std::vector<double> &gainB);

  void initCouplingObject (double &dt, std::vector<double> &gainA, std::vector<double> &gainB);

  void advanceCouplingTerm();

  // add the message information to the reward batch
  void addCurrMessage();

  void starting(const ros::Time&);

  double tau;
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
  // current coupling term per joint
  std::vector<double> couplingTerm;

  // current joint position of the dmp
  std::vector<double> qDmp;

  int scaleCoupling;
  // time at which the coupling term interpolation reaches goal
  double timeCouplingFinal;

   bool flagPubEx  = false;

   std::vector<double> externalForce;

   ros::Duration elapsedTime;

   // callback function reacting to coupling term input
   void ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg);
   ros::Subscriber subCoupling;

   // publisher for reward batches
   ros::Publisher pubBatch;
   // samplesBatch message
   common_msgs::SamplesBatch msgBatch;
   // current CouplingTerm message received
   common_msgs::CouplingTerm msgCoupling;
   // current temporary sample we add to the batch
   common_msgs::MDPSample tempMsg;


  // service client loading robot controllers
  ros::ServiceClient clientLoad;
  // service client switching robot controllers
  ros::ServiceClient clientSwitch;
  // service client unloading robot controllers
  ros::ServiceClient clientUnload;
  // service to call in order to initialize
  controller_manager_msgs::SwitchController srvInit;
  // service to call in order to execute
  controller_manager_msgs::SwitchController srvExec;
  // subscriber to initialized flag
  ros::Subscriber subInit;
  // subscriber to executed flag
  ros::Subscriber subExec;

  ros::Publisher pubExec;

  bool notInit;
  bool exec;
};

#endif // DEBUGDUMMY_H
