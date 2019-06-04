// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <dmpviap_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

//
namespace prcdmp_node {

bool DmpViapController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  robotHardware = robot_hardware;
  nodeHandle = &node_handle;

  initROSCommunication();

  if (!checkRobotSetup()) {return false;}

  int nBFs;
  double dt;
  std::vector<double> initialPosition, goalPosition, gainA, gainB;
  std::vector<std::vector<double>> weights;
  loadDmpData(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  getRobotState();
  std::vector<double> robotQ0(qInit.begin(), qInit.end());
  initDmpObjects(dt, robotQ0, initialPosition, gainA, gainB);

  // publish, so that the state of the robot (initialized or not) is known to the manager
  std_msgs::Bool msg;
  msg.data = notInitializedDMP;
  pub.publish(msg);
  return true;
}

void DmpViapController::starting(const ros::Time& /* time */) {
    ROS_INFO("DmpViapController: starting()");
    //assume initialization
    notInitializedDMP = false;
    flagPubEx = false;

    getRobotState();
    // adapt the dmp to the initial joint positions of the robot
    std::vector<double> qInitV(qInit.begin(), qInit.end());
    dmpInitialize.setInitialPosition(qInitV); // also initializes the dmp trajectory (resetting the canonical sytem)

    dmpInitialize.setFinalPosition(viaPointQ);

    dmpInitialize.resettState();

    elapsedTime = ros::Duration(0.0);
}

void DmpViapController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
    elapsedTime += period;

    std::vector<double> dq(7,0.0000001);

    dq = dmpInitialize.simpleStep(externalForce, tau);

    checkStoppingCondition();

    commandRobot(dq);
}


void DmpViapController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void DmpViapController::initROSCommunication(){
    // publisher to send end of initialization signal
    pub = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_notInit", 10);
}

bool DmpViapController::checkRobotSetup(){
    velocity_joint_interface_ = robotHardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
      ROS_ERROR(
          "DmpViapController: Error getting velocity joint interface from hardware!");
      return false;
    }
    std::vector<std::string> joint_names;
    if (!nodeHandle->getParam("joint_names", joint_names)) {
      ROS_ERROR("DmpViapController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
      ROS_ERROR_STREAM("DmpViapController: Wrong number of joint names, got "
                       << joint_names.size() << " instead of 7 names!");
      return false;
    }
    velocity_joint_handles_.resize(7);
    for (size_t i = 0; i < 7; ++i) {
      try {
        velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
      } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "DmpViapController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR("DmpViapController: Could not get state interface from hardware");
      return false;
    }

    if (!nodeHandle->getParam("/franka_control/robot_ip", robotIp)) {
      ROS_ERROR("Invalid or no robot_ip parameter provided");
      return false;
    }

    return true;
}

bool DmpViapController::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
                                             std::vector<double> &goalv, std::vector<std::vector<double> > &w,
                                             std::vector<double> &gainA, std::vector<double> &gainB){
    //----------------------load DMP specific config data from files----------------------
    std::string datasetPath;
    if (!nodeHandle->getParam("/dmp_velocity_controller/data_set", datasetPath)) {
      ROS_ERROR("DmpViapController: Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
      return false;
    }
    // handles config file access
    std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
    Config config(datasetPath, basePackagePath);
    //fill data from json to variables
    dofs = config.getDmpJson()["dofs"].asInt();
    std::cout<<"DmpViapController: DOFs: "<<dofs<<std::endl;
    nBFs = config.getDmpJson()["n_basis"].asInt();
    dt = config.getDmpJson()["dt"].asDouble();
    tau = 1.0/timeSpan; // TODO: for initialization the timespan can be shorter?!
    // initialize arrays from config file
    moveJsonArrayToVec(config.getDmpJson()["q0"], y0v);
    moveJsonArrayToVec(config.getDmpJson()["goal"], goalv);
    moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
    moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);
    //fill data from json to variables
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    ROS_INFO("DmpCtVelocityController: executing episode #%d",episodeNr);
    config.fillTrajectoryPath(episodeNr);
    return true;
}

bool DmpViapController::checkRobotInit() {
    // check for initial joint positions of the robot
    getRobotState();

    for (size_t i = 0; i < dofs; i++) {
        if (std::abs(qInit[i] - dmpQ0[i]) > 0.05) { //TODO: is this a good threshold?
            ROS_ERROR_STREAM(
                        "DmpViapController: Robot is not in the expected starting position for "
                        "this dmp.");

            //TODO: how to know, we are not running this controller? difference between loading and starting...
            return false;
        }
    }
    return true;
}

bool DmpViapController::getRobotState(){
    // check for initial joint positions of the robot
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpViapController: Could not get state interface from hardware when starting the controller");
    }
    try {
        auto state_handle = state_interface->getHandle("panda_robot");

        for (size_t i = 0; i < dofs; i++) {
            qInit[i] = state_handle.getRobotState().q_d[i];
        }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
                    "DmpViapController: Exception getting state handle: " << e.what());
        return false;
    }
    return true;
}

void DmpViapController::initDmpObjects( double &dt, std::vector<double> &initialPosition,
                                           std::vector<double> &goalPosition,
                                           std::vector<double> &gainA, std::vector<double> &gainB) {
    dmpInitialize = DiscreteDMP(dofs, dt, initialPosition, goalPosition, gainA, gainB);
}

void DmpViapController::checkStoppingCondition(){
    if (dmpInitialize.getTrajFinished()) {
        notInitializedDMP = false;

        //TODO: publish the changed states to a topic so that the controller_manager can switch controllers
        if (!flagPubEx) {
            ROS_INFO("DmpViapController: finished the target trajectory after time[s]: [%f]", elapsedTime.toSec());
            std_msgs::Bool msg;
            msg.data = notInitializedDMP;
            pub.publish(msg);
            flagPubEx = true;
        }
    }
}

void DmpViapController::commandRobot(const std::vector<double> &dq){
    double omega = 0.0;
    int it = 0;
    for (auto joint_handle : velocity_joint_handles_) {
        omega = dq[it];
        joint_handle.setCommand(omega);
        it++;
    }
}

}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpViapController,
                       controller_interface::ControllerBase)
