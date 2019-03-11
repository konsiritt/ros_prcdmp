
#include <serviceDummy.h>

// notes: idea for problems:
//        - right now i am starting and stopping at the same time, initially no controller can be stopped though
//        -

serviceDummy::serviceDummy(ros::NodeHandle &node) {

  std::cout<<"serviceDummy: in the constructor now"<<std::endl;

  nodeHandle = node;

  subInit = node.subscribe("/prcdmp/flag_notInit", 10, &serviceDummy::initializedCallback, this);

  subExec = node.subscribe("/prcdmp/flag_exec", 10, &serviceDummy::executedCallback, this);

  clientLoad = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
  clientUnload = node.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/unload_controller");
  clientSwitch = node.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");

  pubExec = node.advertise<std_msgs::Bool>("/prcdmp/flag_exec", 10);

  notInit = false;

  /*
  // load controllers initially
  controller_manager_msgs::LoadController srv;
  srv.request.name = "dmpstart_velocity_controller";
  if (clientLoad.call(srv)) {
    ROS_INFO("dmpstart_velocity_controller: successfully loaded");
  }
  if (srv.response.ok){
    std::cout<<"serviceDummy: response to load service of dmpstart_velocity_controller is ok"<<std::endl;
  }
  else {
    std::cout<<"serviceDummy: response to load service of dmpstart_velocity_controller is NOT ok"<<std::endl;
  }

  srv.request.name = "dmp_velocity_controller";
  if (clientLoad.call(srv)) {
    ROS_INFO("dmp_velocity_controller: successfully loaded");
  }
  if (srv.response.ok){
    std::cout<<"serviceDummy: response to load service of dmp_velocity_controller is ok"<<std::endl;
  }
  else {
    std::cout<<"serviceDummy: response to load service of dmp_velocity_controller is NOT ok"<<std::endl;
  }
  */

  // setting up services to call for switching controllers
  std::vector < std::string > start_controllers;
  start_controllers.push_back("dmpstart_velocity_controller");
  std::vector < std::string > stop_controllers;
  stop_controllers.push_back("dmp_velocity_controller");
  srvInit.request.start_controllers = start_controllers;
  srvInit.request.stop_controllers = stop_controllers;
  srvInit.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT; // accepts errors

  srvExec.request.start_controllers = stop_controllers;
  srvExec.request.stop_controllers = start_controllers;
  srvExec.request.strictness = controller_manager_msgs::SwitchController::Request::BEST_EFFORT; // accepts errors

  std::cout<<"serviceDummy: DONE in the constructor now"<<std::endl;
}

serviceDummy::~serviceDummy(){

  //TODO: stop all running controllers before unloading

  controller_manager_msgs::UnloadController srv;
  srv.request.name = "dmpstart_velocity_controller";
  if ( clientUnload.call(srv) ) {
    ROS_INFO("dmpstart_velocity_controller: successfully unloaded");
  }
  srv.request.name = "dmp_velocity_controller";
  if ( clientUnload.call(srv) ) {
    ROS_INFO("dmp_velocity_controller: successfully unloaded");
  }
}

void serviceDummy::initializedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) {
    //TODO: this is the case that is switched upon other conditions, i.e. is the learner ready etc.
    //std_msgs::Bool msg;
    //msg.data = true;
    //pubExec.publish(msg);

    notInit = msg->data;

    ros::Duration(1.5).sleep();

    ROS_INFO("switching controllers now to start execution of DMP controller dmp_velocity_controller");
    if (!clientSwitch.call(srvExec)) {
      ROS_INFO("switch controller service call failed in initializedCallback");
    }
    // debugging
    if (srvExec.response.ok){
      std::cout<<"serviceDummy: response to switch service of dmp_velocity_controller is ok"<<std::endl;
    }
    else {
      std::cout<<"serviceDummy: response to switch service of dmp_velocity_controller is NOT ok"<<std::endl;
    }
  }
  else if (notInit!=msg->data){
    notInit = msg->data;
    // state: not initialized, also set executeDMP to false, which in turn will initialize the robot
    std_msgs::Bool msg;
    msg.data = false;
    pubExec.publish(msg);
  }
  else {
  }
}

void serviceDummy::executedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) {
    ros::Duration(1.5).sleep();

    ROS_INFO("switching controllers now to return robot into starting pose");
    if (!clientSwitch.call(srvInit)) {
      ROS_INFO("switch controller service call failed in executedCallback");
    }
    // debugging
    if (srvInit.response.ok){
      std::cout<<"serviceDummy: response to switch service of dmpstart_velocity_controller is ok"<<std::endl;
    }
    else {
      std::cout<<"serviceDummy: response to switch service of dmpstart_velocity_controller is NOT ok"<<std::endl;
    }
  }
}

int main(int argc, char **argv)
/*
 * 1. Initialize the ROS system
 * 2. Advertize that we are publishing on the chatter topic to the master
 * 3. Loop while publishing messages to chatter 10 times a second
 */
{
  ros::init(argc, argv, "serviceDummy");

  // NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle n;

  // initializes Service dummy, using callbacks to topics to switch controllers
  serviceDummy serviceMngr(n);


  /*
  srv_list_controllers_ = cm_node_.advertiseService("list_controllers", &ControllerManager::listControllersSrv, this);
    srv_list_controller_types_ = cm_node_.advertiseService("list_controller_types", &ControllerManager::listControllerTypesSrv, this);
    srv_load_controller_ = cm_node_.advertiseService("load_controller", &ControllerManager::loadControllerSrv, this);
    srv_unload_controller_ = cm_node_.advertiseService("unload_controller", &ControllerManager::unloadControllerSrv, this);
    srv_switch_controller_ = cm_node_.advertiseService("switch_controller", &ControllerManager::switchControllerSrv, this);
  srv_reload_libraries_ = cm_node_.advertiseService("reload_controller_libraries", &ControllerManager::reloadControllerLibrariesSrv, this);
  */

  // loop frequency of 1 Hz
  ros::Rate loop_rate(10);

  // ros::ok == False handles interruptions
  int count = 0;
  while (ros::ok())
  {
    // to add possible subscription when receiving a callback
    ros::spinOnce();

    //use the loop rate object to get the desired rate
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
