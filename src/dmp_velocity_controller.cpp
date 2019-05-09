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

  initROSCommunication();

  if (!checkRobotSetup()) {return false;}


  //----------------------load DMP specific config data from files----------------------
  int nBFs;
  double dt;
  std::vector<double> initialPosition, goalPosition, gainA, gainB;
  std::vector<std::vector<double>> weights;
  loadDmpData(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  //----------------------initialize dmp runtime object----------------------
  initDmpObjects(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  if (!checkRobotInit()) {return false;}

  return true;
}

void DmpVelocityController::initROSCommunication(){
    pubExec = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_exec", 10);

    // subscriber that handles changes to the dmp coupling term: TODO: change to coupling term
    subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 1, &DmpVelocityController::ctCallback, this);
    // subscriber that handles changes to the smoothed dmp coupling term
    subCouplingSmoothed = nodeHandle->subscribe("/coupling_term_estimator/coupling_term/smoothed", 1, &DmpVelocityController::ctSmoothedCallback, this);
}

bool DmpVelocityController::checkRobotSetup(){
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

    if (!nodeHandle->getParam("/franka_control/robot_ip", robotIp)) {
        ROS_ERROR("Invalid or no robot_ip parameter provided");
        return false;
    }
    return true;
}

bool DmpVelocityController::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
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
    Config config(datasetPath, basePackagePath);
    //------fill data from json to variables----------------------
    dofs = config.getDmpJson()["dofs"].asInt();
    std::cout<<"DmpCtVelocityController: DOFs: "<<dofs<<std::endl;
    nBFs = config.getDmpJson()["n_basis"].asInt();
    dt = config.getDmpJson()["dt"].asDouble();
    double timeSpan = config.getDmpJson()["timespan"].asDouble();
    tau = 1.0/timeSpan;
    //------initialize arrays from config file----------------------
    moveJsonArrayToVec(config.getDmpJson()["q0"], y0v);
    moveJsonArrayToVec(config.getDmpJson()["goal"], goalv);
    moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
    moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);
    //------fill data from json to variables----------------------
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    ROS_INFO("DmpCtVelocityController: executing episode #%d",episodeNr);
    config.fillTrajectoryPath(episodeNr);
    if (episodeNr ==0) {
        UTILS::loadWeights(config.getInitialWPath(),w);
    }
    else {
        UTILS::loadWeights(config.getwPath(),w);
    }

    return true;
}

void DmpVelocityController::initDmpObjects(int &nBFs, double &dt, std::vector<double> &initialPosition,
                                             std::vector<double> &goalPosition, std::vector<std::vector<double> > &weights,
                                             std::vector<double> &gainA, std::vector<double> &gainB) {
    //DiscreteDMP dmpTemp(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
    dmp = DiscreteDMP(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
}

bool DmpVelocityController::checkRobotInit() {
    // check for initial joint positions of the robot
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR("DmpVelocityController: Could not get state interface from hardware when starting the controller");
    }
    try {
      auto state_handle = state_interface->getHandle("panda_robot");

      for (size_t i = 0; i < q0.size(); i++) {
        qInit[i] = state_handle.getRobotState().q_d[i];
        // if only just one joint is not close enough to q0, assume robot is not initialized
        if (std::abs(qInit[i] - q0[i]) > 0.05) { //TODO: is this a good threshold?
          ROS_ERROR_STREAM(
              "DmpVelocityController: Robot is not in the expected starting position for "
              "this dmp.");

          //TODO: how to know, we are not running this controller? difference between loading and starting...
          //return false;
        }
      }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "DmpVelocityController: Exception getting state handle: " << e.what());
      return false;
    }
    return true;
}

void DmpVelocityController::starting(const ros::Time& /* time */) {
  std::cout<<"DmpVelocityController: starting()"<<std::endl;
  // initialize the dmp trajectory (resetting the canonical sytem)
  dmp.resettState(); 
  //assume initialization 
  flagPubEx = false;

  checkRobotInit();

  elapsedTime = ros::Duration(0.0);
}

void DmpVelocityController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsedTime += period;

  std::vector<double> dq(7,0.0000001);

  dq = dmp.step(externalForce, tau);

  checkStoppingCondition();
  
  commandRobot(dq);
}

void DmpVelocityController::checkStoppingCondition(){
    //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities
    if (dmp.getTrajFinished()) {
        //publish the changed states to a topic so that the controller_manager can switch controllers
        if (!flagPubEx) {
            ROS_INFO("DmpCtVelocityController: finished the target trajectory after time[s]: [%f]", elapsedTime.toSec());
            std_msgs::Bool msg;
            msg.data = false;
            pubExec.publish(msg);
            flagPubEx = true;
        }
    }
}

void DmpVelocityController::commandRobot(const std::vector<double> &dq){
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
void DmpVelocityController::ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {

}
void DmpVelocityController::ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    std::vector<double> couplings;
    couplings.push_back(*msg->data.data());
    dmp.setCouplingTerm(couplings);
}


}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpVelocityController,
                       controller_interface::ControllerBase)
