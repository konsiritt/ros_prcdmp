// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <dmp_ct_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace prcdmp_node {

bool DmpCtVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  robotHardware = robot_hardware;
  nodeHandle = &node_handle;

  if (!checkRobotSetup()) {return false;}

 //----------------------load DMP specific config data from files----------------------
  int dofs, nBFs;
  double dt;
  std::vector<double> initialPosition, goalPosition, gainA, gainB;
  std::vector<std::vector<double>> weights ;
  loadDmpData(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  //----------------------initialize dmp runtime object----------------------
  initDmpObjects(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  initROSCommunication();
  std::vector<double> initCoupling(dofs, 0.0);
  std::vector<double> goalCoupling(dofs, 0.0);
  scaleCoupling = 10; // i.e. 10 steps of coulingDmp per dmp step
  DiscreteDMP dmpTemp2(dofs,dt/scaleCoupling,initCoupling,goalCoupling,gainA,gainB);
  couplingDmp = dmpTemp2;

  //set current coupling term to zero
  std::vector<double> tempVec(q0.size(),0.0);
  couplingTerm = tempVec;

  ROS_INFO("DmpCtVelocityController: setup DMP objects");

  auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
  // check for initial joint positions of the robot
  ROS_INFO("DmpCtVelocityController: checking for initial joint positions of the robot");
  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    for (size_t i = 0; i < q0.size(); i++) {
      qInit[i] = state_handle.getRobotState().q_d[i];
      // if only just one joint is not close enough to q0, assume robot is not initialized
      if (std::abs(qInit[i] - q0[i]) > 0.05) { //TODO: is this a good threshold?
        notInitializedDMP = true;
        ROS_ERROR_STREAM(
            "DmpCtVelocityController: Robot is not in the expected starting position for "
            "this dmp.");

        //TODO: how to know, we are not running this controller? difference between loading and starting...
        //return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "DmpCtVelocityController: Exception getting state handle: " << e.what());
    return false;
  }

  if (!node_handle.getParam("/franka_control/robot_ip", robotIp)) {
    ROS_ERROR("Invalid or no robot_ip parameter provided");    
    return 1;
  }
  return true;
}


void DmpCtVelocityController::starting(const ros::Time& /* time */) {
  std::cout<<"DmpCtVelocityController: starting()"<<std::endl;
  // initialize the dmp trajectory (resetting the canonical sytem)
  dmp.resettState(); 
  couplingDmp.resettState();

  //reset iterator for reference joint positions
  iterateRef = 0;

  //reset the message batch for reward information
  msgBatch.samples.clear();

  //assume initialization
  notInitializedDMP = false;
  flagPubEx = false;

  //set current coupling term to zero
  std::vector<double> tempVec(q0.size(),0.0);
  couplingTerm = tempVec;

  auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("DmpCtVelocityController: Could not get state interface from hardware when starting the controller");
  }

  // check for initial joint positions of the robot
  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    for (size_t i = 0; i < q0.size(); i++) {
      qInit[i] = state_handle.getRobotState().q_d[i];
      // if only just one joint is not close enough to q0, assume robot is not initialized
      if (std::abs(state_handle.getRobotState().q_d[i] - q0[i]) > 0.05) {
        notInitializedDMP = true;
        std::cout<<"DmpCtVelocityController: joint #"<<i<<" is not in the initial position yet"<<std::endl;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "DmpCtVelocityController: Exception getting state handle: " << e.what());
  }
  elapsed_time_ = ros::Duration(0.0);
}

void DmpCtVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  //advance the coupling term, taking intermediate steps in order to allow faster dynamic
  for (int i=0; i<scaleCoupling; ++i) {
    couplingDmp.simpleStep(externalForce, 100); //TODO: configure this parameter
  }
  couplingTerm = couplingDmp.getY();
  dmp.setCouplingTerm(couplingTerm);
//  std::cout<<"coupling term in update: of size: "<<couplingTerm.size()<< " with elements: ";
//  for (int i = 0; i< couplingTerm.size(); i++) {
//      std::cout<<couplingTerm[i]<<"; ";
//  }
//  std::cout<<std::endl;
  //advance the actual dmp
  std::vector<double> dq(q0.size(),0.0000001);
  dq = dmp.step(externalForce, tau);
  qDmp = dmp.getY();

  //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities
  if (dmp.getTrajFinished()) {
    // done executing the dmp
    executingDMP = false;

    //publish the changed states to a topic so that the controller_manager can switch controllers
    if (!flagPubEx) {
      std::cout<<"DmpCtVelocityController: finished the target trajectory after time[s]: "<< elapsed_time_<<std::endl;
      std_msgs::Bool msg;
      msg.data = executingDMP;
      pubExec.publish(msg);
      flagPubEx = true;
    }
  }

  double omega = 0.0; 
  int it = 0;
  for (auto joint_handle : velocity_joint_handles_) {
    omega = refDmpVel[iterateRef][it];//dq[it]; //
    if (it == 6) {
        omega = dq[it];
    }
    joint_handle.setCommand(omega);
    it++;
  }
  // add the current message for later publishing it
  addCurrMessage();
}

void DmpCtVelocityController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    std::cout<<"publishing the msgBatch of size: "<<msgBatch.samples.size()<< " now" <<std::endl;
    pubBatch.publish(msgBatch);
}

//TODO: adapt to react to a change of the coupling term as a topic
void DmpCtVelocityController::ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
  // reset counter that counts control cycles per ct
  msgCoupling = *msg;
  msgCoupling.header = msg->header;
  msgCoupling.data = msg->data;
  msgCoupling.msg_id = msg->msg_id;
  std::vector<double> temp(msgCoupling.data.begin(),msgCoupling.data.end());
  couplingDmp.setInitialPosition(couplingTerm);
  couplingDmp.setFinalPosition(temp);

//  std::cout<<"coupling term callback: of size: "<<temp.size()<< " with elements: ";
//  for (int i = 0; i< temp.size(); i++) {
//      std::cout<<temp[i]<<"; ";
//  }
//  std::cout<<std::endl;
}

void DmpCtVelocityController::addCurrMessage(){
    common_msgs::MDPSample tempMsg;
    tempMsg.ct = msgCoupling;
    tempMsg.reward = 0;
    tempMsg.mask = 0;
    boost::array<double,7> tempArray = {0};
    for (int i=0; i < qDmp.size(); ++i) {
        tempArray[i] = refDmpTraj[iterateRef][i] - qDmp[i];
    }
    iterateRef++;
    tempMsg.q_offset = tempArray;
    msgBatch.samples.push_back(tempMsg);

//    std::cout<<"addCurrMessage, control cycles per ct: "<<iterateCt<<" with ct: ";
//    for (int i = 0; i< tempMsg.ct.data.size(); i++) {
//        std::cout<<tempMsg.ct.data[i]<<"; ";
//    }
//    std::cout<<std::endl;
}

void DmpCtVelocityController::initROSCommunication(){
    //----------------------initialize the publishing nodes----------------------
    pubExec = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_exec", 1000);
    pubBatch = nodeHandle->advertise<common_msgs::SamplesBatch>("/prcdmp/episodic_batch", 1000);

    //----------------------subscriber that handles changes to the dmp coupling term----------------------
    subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 100, &DmpCtVelocityController::ctCallback, this);
}

bool DmpCtVelocityController::checkRobotSetup(){
    //----------------------do some robot hardware initializing work----------------------
    velocity_joint_interface_ = robotHardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
      ROS_ERROR(
          "DmpCtVelocityController: Error getting velocity joint interface from hardware!");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!nodeHandle->getParam("joint_names", joint_names)) {
      ROS_ERROR("DmpCtVelocityController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
      ROS_ERROR_STREAM("DmpCtVelocityController: Wrong number of joint names, got "
                       << joint_names.size() << " instead of 7 names!");
      return false;
    }
    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
      try {
        velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "DmpCtVelocityController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR("DmpCtVelocityController: Could not get state interface from hardware");
      return false;
    }
    return true;
}

bool DmpCtVelocityController::loadDmpData(int &dofs, int &nBFs, double &dt, std::vector<double> &y0v,
                                          std::vector<double> &goalv, std::vector<std::vector<double> > &w,
                                          std::vector<double> &gainA, std::vector<double> &gainB) {
    //----------------------load DMP specific config data from files----------------------
    std::string datasetPath;
    if (!nodeHandle->getParam("/dmp_velocity_controller/data_set", datasetPath)) {
      ROS_ERROR("DmpCtVelocityController: Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
      return false;
    }
    //-------handles config file access----------------------
    std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
    std::cout<<"DmpCtVelocityController: this is the package base path: "<<basePackagePath<<std::endl;
    Config config(datasetPath, basePackagePath);
    std::cout<<"DmpCtVelocityController: config file has been created"<<std::endl;
    //------fill data from json to variables----------------------
    dofs = config.getDmpJson()["dofs"].asInt();
    std::cout<<"DmpCtVelocityController: DOFs: "<<dofs<<std::endl;
    nBFs = config.getDmpJson()["n_basis"].asInt();
    dt = config.getDmpJson()["dt"].asDouble();
    double timeSpan = config.getDmpJson()["timespan"].asDouble();
    tau = 1.0/timeSpan;
    //------initialize arrays from config file----------------------
    std::array<double,7> goal;
    moveJsonArrayToVec(config.getDmpJson()["q0"], q0);
    moveJsonArrayToVec(config.getDmpJson()["goal"], goal);
    moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
    moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);
    //------fill data from json to variables----------------------
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    std::cout<<"DmpCtVelocityController: executing episode #"<<episodeNr<<std::endl;
    config.fillTrajectoryPath(episodeNr);
    if (episodeNr ==0) {
        UTILS::loadWeights(config.getInitialWPath(),w);
    }
    else {
        UTILS::loadWeights(config.getwPath(),w);
    }
    //------convert arrays to vectors----------------------
    std::vector<double> y0vTemp(q0.begin(), q0.end());
    std::vector<double> goalvTemp(goal.begin(), goal.end());
    y0v = y0vTemp;
    goalv = goalvTemp;

    return true;
}

void DmpCtVelocityController::initDmpObjects(int &dofs, int &nBFs, double &dt, std::vector<double> &initialPosition,
                                          std::vector<double> &goalPosition, std::vector<std::vector<double> > &weights,
                                          std::vector<double> &gainA, std::vector<double> &gainB) {
    DiscreteDMP dmpTemp(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
    dmp = dmpTemp;
    std::vector<std::vector<double>> dummY;
    dmp.rollout(refDmpTraj,refDmpVel,dummY,externalForce,tau, -1, 0);
    dmp.resettState();
}

}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpCtVelocityController,
                       controller_interface::ControllerBase)
