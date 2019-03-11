#ifndef SERVICEDUMMY_H
#define SERVICEDUMMY_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/UnloadController.h>

class serviceDummy
{

public:
  serviceDummy(ros::NodeHandle &node);

  ~serviceDummy();

  ros::NodeHandle nodeHandle;

private:
  void initializedCallback(const std_msgs::Bool::ConstPtr& msg);
  void executedCallback(const std_msgs::Bool::ConstPtr& msg);

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



#endif // SERVICEDUMMY_H
