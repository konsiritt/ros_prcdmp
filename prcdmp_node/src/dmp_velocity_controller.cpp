// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <dmp_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace prcdmp_node {

bool DmpVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  robotHardware = robot_hardware;
  nodeHandle = &node_handle;

  pubInit = node_handle.advertise<std_msgs::Bool>("/prcdmp/flag_notInit", 10);
  pubExec = node_handle.advertise<std_msgs::Bool>("/prcdmp/flag_exec", 10);

  // subscriber that handles changes to the dmp coupling term
  subCoupling = node_handle.subscribe("/prcdmp/coupling_term", 10, &DmpVelocityController::callback, this);

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "DmpVelocityController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("DmpVelocityController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("DmpVelocityController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DmpVelocityController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("DmpVelocityController: Could not get state interface from hardware");
    return false;
  }


  std::string datasetPath;
  if (!node_handle.getParam("/dmp_velocity_controller/data_set", datasetPath)) {
    ROS_ERROR("Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
    return 1;
  }

  /// load DMP specific config data from files

  // handles config file access
  std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
  std::cout<<"DmpVelocityController: this is the package base path: "<<basePackagePath<<std::endl;
  Config config(datasetPath, basePackagePath);
  std::cout<<"DmpVelocityController: config file has been created"<<std::endl;

  //fill data from json to variables
  int dofs = config.getDmpJson()["dofs"].asInt();
  std::cout<<"DmpVelocityController: DOFs: "<<dofs<<std::endl;
  int nBFs = config.getDmpJson()["n_basis"].asInt();
  double dt = config.getDmpJson()["dt"].asDouble();
  double timeSpan = config.getDmpJson()["timespan"].asDouble();
  tau = 1.0/timeSpan;

  // initialize arrays from config file
  std::array<double,7> goal;
  std::vector<double> gainA, gainB;
  moveJsonArrayToVec(config.getDmpJson()["q0"], q0);
  moveJsonArrayToVec(config.getDmpJson()["goal"], goal);
  moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
  moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);

  //fill data from json to variables

  int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
  std::cout<<"DmpVelocityController: executing episode #"<<episodeNr<<std::endl;
  config.fillTrajectoryPath(episodeNr);

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

  // initialize dmp runtime object
  DiscreteDMP dmpTemp(dofs, nBFs, dt, y0v, goalv, w, gainA, gainB);
  dmp = dmpTemp;

  // check for initial joint positions of the robot
  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    for (size_t i = 0; i < q0.size(); i++) {
      qInit[i] = state_handle.getRobotState().q_d[i];
      // if only just one joint is not close enough to q0, assume robot is not initialized
      if (std::abs(qInit[i] - q0[i]) > 0.05) { //TODO: is this a good threshold?
        notInitializedDMP = true;
        ROS_ERROR_STREAM(
            "DmpVelocityController: Robot is not in the expected starting position for "
            "this dmp.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "DmpVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  if (!node_handle.getParam("/franka_control/robot_ip", robotIp)) {
    ROS_ERROR("Invalid or no robot_ip parameter provided");    
    return 1;
  }
  return true;
}


void DmpVelocityController::starting(const ros::Time& /* time */) {
  std::cout<<"DmpVelocityController: starting()"<<std::endl;
  // initialize the dmp trajectory (resetting the canonical sytem)
  dmp.resettState(); 
  //assume initialization 
  notInitializedDMP = false;

  auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("DmpVelocityController: Could not get state interface from hardware when starting the controller");
  }

  // check for initial joint positions of the robot
  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    for (size_t i = 0; i < q0.size(); i++) {
      qInit[i] = state_handle.getRobotState().q_d[i];
      // if only just one joint is not close enough to q0, assume robot is not initialized
      if (std::abs(state_handle.getRobotState().q_d[i] - q0[i]) > 0.05) {
        notInitializedDMP = true;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "DmpVelocityController: Exception getting state handle: " << e.what());
  }
  elapsed_time_ = ros::Duration(0.0);
}

void DmpVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  std::vector<double> dq(7,0.0000001);

if (executingDMP){
    dmp.step(externalForce, tau);
    dq = dmp.getDY();
    if (dmp.getTrajFinished()) {
      //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities

      std::cout<<"DmpVelocityController: finished the target trajectory after time[s]: "<< elapsed_time_<<std::endl;
      // done executing the dmp
      executingDMP = false;
      // joints are not in initial position anymore
      notInitializedDMP = true;      

      //publish the changed states to a topic so that the controller_manager can switch controllers
      std_msgs::Bool msg;
      msg.data = executingDMP;
      pubExec.publish(msg);
      msg.data = notInitializedDMP;
      pubInit.publish(msg);
    }
  }
  else {
  }
  
  double omega = 0.0; 
  int it = 0;
  for (auto joint_handle : velocity_joint_handles_) {
    omega = dq[it];
    joint_handle.setCommand(omega);
    it++;
  }
}

void DmpVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

//TODO: adapt to react to a change of the coupling term as a topic
void DmpVelocityController::callback(const std_msgs::Bool::ConstPtr& msg) {
  this->executingDMP = msg->data;
}

}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpVelocityController,
                       controller_interface::ControllerBase)
