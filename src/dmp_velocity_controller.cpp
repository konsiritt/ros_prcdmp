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

    // initialize dmpGoal to values from file
    for (int iter=0;iter<dmpGoal.size();iter++){
        dmpGoal[iter] = goalPosition[iter];
    }

    // unroll dmp in refQ, resetState for DMP
    std::vector<std::vector<double>> refDQ, refDDQ;
    dmp.rollout(refQ, refDQ, refDDQ,externalForce,tau, -1, 0);
    refDQ.clear();
    refDDQ.clear();

    saveRobotState();

    setupSampling();
    return true;
}

void DmpVelocityController::starting(const ros::Time& /* time */) {
    ROS_INFO("DmpVelocityController: including ct: starting()");
    // initialize the dmp trajectory (resetting the canonical sytem)
    dmp.resettState();
    //assume initialization
    flagPubEx = false;
    flagPubErr = false;
    refIter = 0;

    saveRobotState();

    updateDmpInit();

    sampleGoalQ();

    // create new MDPbatch
    ctBatch.samples.clear();
    ctBatch.goal = dmpGoal;

    firstCB = true;

    if (logging) {
        saveDmpQ.clear();
        saveRobotQ.clear();
        saveTime.clear();
    }


    elapsedTime = ros::Duration(0.0);
}

void DmpVelocityController::update(const ros::Time& /* time */,
                                   const ros::Duration& period) {
    elapsedTime += period;

    std::vector<double> dq(7,0.0000001);

    dq = dmp.step(externalForce, tau);
    refIter++;

    if (logging) {
        std::vector <double> tempQ = dmp.getY();
        saveDmpQ.push_back(tempQ);
        saveRobotState();
        std::copy(robotQ.begin(),robotQ.end(),tempQ.begin());
        saveRobotQ.push_back(tempQ);
        saveTime.push_back(elapsedTime.toSec());
    }

    checkRobotState();

    checkStoppingCondition();

    isValidVelocity(dq);

    commandRobot(dq);
}

void DmpVelocityController::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    //if (ctBatch.samples.size() > 30){

    computeGoalOffset();
    pubBatch.publish(ctBatch);
    ROS_INFO("DmpVelocityController: batch size published: #%d", ctBatch.samples.size());
    //}
    refIter = -1;
    checkRobotState();

    if (logging){
        ROS_INFO("logging is on: dumping last dmp trajectory to file");
        saveDmpData();
    }
}

void DmpVelocityController::initROSCommunication(){
    pubExec = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_exec", 10);
    pubError = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/error_occured", 10);
    pubBatch = nodeHandle->advertise<common_msgs::SamplesBatch>("/prcdmp/episodic_batch", 10);
    pubGoal = nodeHandle->advertise<std_msgs::Float64MultiArray>("/prcdmp/q_goal", 10);

    // subscriber that handles changes to the dmp coupling term: TODO: change to coupling term
    subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 1, &DmpVelocityController::ctCallback, this);
    // subscriber that handles changes to the smoothed dmp coupling term
    subCouplingSmoothed = nodeHandle->subscribe("/coupling_term_estimator/coupling_term/smoothed", 1, &DmpVelocityController::ctSmoothedCallback, this);

    subFrankaStates = nodeHandle->subscribe("/franka_state_controller/franka_states", 1, &DmpVelocityController::frankaStateCallback, this);

    if (!nodeHandle->getParam("/dmp_velocity_controller/std_offset_q0", stdOffset)) {
      ROS_ERROR("DmpStartVelocityController: Invalid or no std_offset_q0 parameter provided; provide e.g. std_offset_q0:=0.05");
    }

    if (!nodeHandle->getParam("/dmp_velocity_controller/logging", logging)) {
      ROS_ERROR("DmpStartVelocityController: Invalid or no logging parameter provided; provide e.g. logging:=true");
    }

    collisionClient = nodeHandle->serviceClient<franka_control::SetForceTorqueCollisionBehavior>("/franka_control/set_force_torque_collision_behavior");
}

bool DmpVelocityController::checkRobotSetup(){
    //----------------------do some robot hardware initializing work----------------------
    velocity_joint_interface_ = robotHardware->get<hardware_interface::VelocityJointInterface>();
    if (velocity_joint_interface_ == nullptr) {
        ROS_ERROR(
                    "DmpVelocityController: Error getting velocity joint interface from hardware!");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!nodeHandle->getParam("joint_names", joint_names)) {
        ROS_ERROR("DmpVelocityController: Could not parse joint names");
    }
    if (joint_names.size() != 7) {
        ROS_ERROR_STREAM("DmpVelocityController: Wrong number of joint names, got "
                         << joint_names.size() << " instead of 7 names!");
        return false;
    }
    velocity_joint_handles_.resize(7);
    for (size_t iter = 0; iter < 7; ++iter) {
        try {
            velocity_joint_handles_[iter] = velocity_joint_interface_->getHandle(joint_names[iter]);
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                        "DmpVelocityController: Exception getting joint handles: " << ex.what());
            return false;
        }
    }
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpVelocityController: Could not get state interface from hardware");
        return false;
    }

    if (!nodeHandle->getParam("/franka_control/robot_ip", robotIp)) {
        ROS_ERROR("DmpVelocityController: Invalid or no robot_ip parameter provided");
        return false;
    }

    modelInterface = robotHardware->get<franka_hw::FrankaModelInterface>();
    if (modelInterface == nullptr) {
      ROS_ERROR_STREAM("DmpVelocityController: Error getting model interface from hardware");
      for (int retry = 0; retry <60; retry++) {
          modelInterface = robotHardware->get<franka_hw::FrankaModelInterface>();
          if (modelInterface == nullptr) {
            ROS_ERROR_STREAM("DmpVelocityController: Error getting model interface from hardware");
          }
          else break;
      }
      return false;
    }

    std::string armId;
    if (!nodeHandle->getParam("arm_id", armId)) {
      ROS_ERROR("DmpVelocityController: Could not read parameter arm_id");
      return false;
    }

    try {
      modelHandle = std::make_unique<franka_hw::FrankaModelHandle>(
          modelInterface->getHandle(armId + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DmpVelocityController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    // setup collision behavior
    franka_control::SetForceTorqueCollisionBehavior collisionSrv;
    collisionSrv.request.lower_torque_thresholds_nominal = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    collisionSrv.request.upper_torque_thresholds_nominal = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    collisionSrv.request.lower_force_thresholds_nominal = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    collisionSrv.request.upper_force_thresholds_nominal = {25.0, 25.0, 25.0, 25.0, 25.0, 25.0};
    if (collisionClient.call(collisionSrv)) {
        ROS_INFO("DmpVelocityController: Collision information successfully set");
    }
    else {
        return false;
    }

    return true;
}

bool DmpVelocityController::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
                                        std::vector<double> &goalv, std::vector<std::vector<double> > &w,
                                        std::vector<double> &gainA, std::vector<double> &gainB) {
    //----------------------load DMP specific config data from files----------------------
    if (!nodeHandle->getParam("/dmp_velocity_controller/data_set", datasetPath)) {
        ROS_ERROR("DmpVelocityController: Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
        return false;
    }
    //-------handles config file access----------------------
    std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
    Config config(datasetPath, basePackagePath);
    //------fill data from json to variables----------------------
    dofs = config.getDmpJson()["dofs"].asInt();
    std::cout<<"DmpVelocityController: DOFs: "<<dofs<<std::endl;
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
    ROS_INFO("DmpVelocityController: executing episode #%d",episodeNr);
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
    saveRobotState();

    for (size_t iter = 0; iter < dofs; iter++) {
        // if only just one joint is not close enough to q0, assume robot is not initialized
        if (std::abs(robotQ[iter] - dmpQ0[iter]) > 0.05) { //TODO: is this a good threshold?
            ROS_ERROR_STREAM(
                        "DmpVelocityController: Robot is not in the expected starting position for "
                        "this dmp.");
            //TODO: how to know, we are not running this controller? difference between loading and starting...
            return false;
        }
    }
    return true;
}

void DmpVelocityController::checkRobotState() {
    if (currentRobotMode != 1 && currentRobotMode != 2) {
        switch (currentRobotMode){
        case 0:
            ROS_INFO("ROBOT_MODE_OTHER=0");
            break;
        case 3:
            ROS_INFO("ROBOT_MODE_GUIDING=3");
            break;
        case 4:
            ROS_INFO("ROBOT_MODE_REFLEX=4: Attempting error recovery!");            
            if (errorRecovery())
            {
                if (!flagPubErr) {
                    std_msgs::Bool msg;
                    msg.data = true;
                    pubError.publish(msg);
                    ROS_INFO("error recovery successful");
                    flagPubErr = true;
                }

            };
            break;
        case 5:
            ROS_INFO("ROBOT_MODE_USER_STOPPED=5");
            break;
        case 6:
            ROS_INFO("ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6");
            break;
        }
    }
}

bool DmpVelocityController::saveRobotState(){
    // check for initial joint positions of the robot
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpVelocityController: Could not get state interface from hardware when starting the controller");
    }
    try {
        auto state_handle = state_interface->getHandle("panda_robot");

        for (size_t iter = 0; iter < dofs; iter++) {
            robotQ[iter] = state_handle.getRobotState().q[iter];
        }
    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
                    "DmpVelocityController: Exception getting state handle: " << e.what());
        return false;
    }
    return true;
}

void DmpVelocityController::checkStoppingCondition(){
    //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities
    if (dmp.getTrajFinished()) {
        //publish the changed states to a topic so that the controller_manager can switch controllers
        if (!flagPubEx) {
            ROS_INFO("DmpVelocityController: finished the target trajectory after time[s]: [%f]", elapsedTime.toSec());
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

bool DmpVelocityController::errorRecovery(){
    actionlib::SimpleActionClient<franka_control::ErrorRecoveryAction> tempClient("/franka_control/error_recovery/", true);
    tempClient.waitForServer();
    franka_control::ErrorRecoveryGoal goalRecovery;
    tempClient.sendGoal(goalRecovery);
    tempClient.waitForResult(ros::Duration(1.0));
    if (tempClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("error recovery successfull");
        return true;
    }
    else
    {
        ROS_INFO("error recovery unsuccessfull");
        return false;
    }
}

//TODO: adapt to react to a change of the coupling term as a topic
void DmpVelocityController::ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    // do not listen, when the controller is done (flagPubEx) or when it is not actie (refIter =-1)
    if (!flagPubEx && refIter >= 0) {

        if (refIter > 0 && !firstCB)
        {
            std::vector<double> currQ = dmp.getY();
            for (int iter=0; iter<dofs; iter++)
            {
                if (refIter-1 > refQ.size())
                {
                    ROS_INFO("DmpVelocityController: we are accessing the reference trajectory out of bounds");
                    ctSample.q_offset[iter] = 0.0;
                }
                else {
                    ctSample.q_offset[iter] = abs(currQ[iter] - refQ[refIter-1][iter]);
                }
            }
            ctBatch.samples.push_back(ctSample);
        }

        // reset for next sample
        ctSample.mask = 0;
        ctSample.reward = 0.0;
        ctSample.ct = *msg;

        firstCB = false;
    }

}

void DmpVelocityController::ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    std::vector<double> coupling(msg->data.begin(),msg->data.end());
    dmp.setCouplingTerm(coupling);
}

void DmpVelocityController::frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg) {
    currentRobotMode = msg->robot_mode;
}

bool DmpVelocityController::isValidVelocity(std::vector<double> velocitiesToApply) {
    // check for initial joint positions of the robot
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpVelocityController: Could not get state interface from hardware when checking velocities");
    }
    std::array<double, 7> currentPos;
    std::array<double, 16> F_T_EE;
    std::array<double, 16> EE_T_K;
    std::array<double, 16> futurePosEnd;
    std::array<double, 16> futurePosFlange;
    std::array<double, 16> futurePosJoint7;
    std::array<double, 16> futurePosJoint6;

    try {
        auto state_handle = state_interface->getHandle("panda_robot");

        currentPos = state_handle.getRobotState().q; //was q_d before
        F_T_EE = state_handle.getRobotState().F_T_EE;
        EE_T_K = state_handle.getRobotState().EE_T_K;

    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
                    "DmpVelocityController: Exception getting state handle: " << e.what());
        return false;
    }

    for(int iter = 0; iter < dofs; iter++)
    {
      currentPos[iter] += velocitiesToApply[iter]/1000.0;
    }
    futurePosEnd = modelHandle->getPose(franka::Frame::kEndEffector, currentPos, F_T_EE, EE_T_K);
    futurePosFlange = modelHandle->getPose(franka::Frame::kFlange, currentPos, F_T_EE, EE_T_K);
    futurePosJoint7 = modelHandle->getPose(franka::Frame::kJoint7, currentPos, F_T_EE, EE_T_K);
    futurePosJoint6 = modelHandle->getPose(franka::Frame::kJoint6, currentPos, F_T_EE, EE_T_K);

    // check cartesian position of End effector with virtual wall height of tabletop
    if (futurePosEnd[14] < virtWallZ_EE || futurePosFlange[14] < virtWallZ_F || futurePosJoint7[14] < virtWallZ_J7 || futurePosJoint6[14] < virtWallZ_J6){
        std_msgs::Bool msg;
        msg.data = false;
        pubError.publish(msg);
        ROS_INFO("DmpVelocityController: virtual wall hit");
    }

    return true;
}

void DmpVelocityController::setupSampling(){
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = std::default_random_engine (seed);
    distribution = std::normal_distribution<double>(meanOffset,stdOffset);
}

std::vector <double> DmpVelocityController::getRandomVectorOffset(){
    std::vector<double> randomVector(7,0.0);
    for (int iterate=0; iterate<dofs; iterate++){
        randomVector[iterate] = distribution(generator);
    }
    return randomVector;
}

std::vector<double> DmpVelocityController::addVectors(const std::vector<double> &element1, const std::vector<double> &element2){
    assert(element1.size() == element2.size());
    std::vector<double> returnVector(element1.size(),0.0);
    for (int iterator=0; iterator < element1.size(); iterator++) {
        returnVector[iterator] = element1[iterator] + element2[iterator];
    }
    return returnVector;
}

void DmpVelocityController::updateDmpInit(){
    std::vector<double> tempQ(robotQ.begin(),robotQ.end());
    dmp.setInitialPosition(tempQ);
    dmp.resettState();
}

void DmpVelocityController::sampleGoalQ(){
    // adapt the dmp to the initial joint positions of the robot
    std::vector<double> dmpGoalV(dmpGoal.begin(), dmpGoal.end());
    std::vector<double> offsetGoalV = getRandomVectorOffset();
    std::vector<double> qGoalWithOffsetV = addVectors(dmpGoalV,offsetGoalV);
    for (int iter=0;iter<dmpGoal.size();iter++){
        dmpGoal[iter] = qGoalWithOffsetV[iter];
    }
    dmp.setFinalPosition(qGoalWithOffsetV); // also initializes the dmp trajectory (resetting the canonical sytem)
    dmp.resettState();

    std_msgs::Float64MultiArray tempMsg;
    std::vector <double> tempGoal (dmpGoal.begin(),dmpGoal.end());
    tempMsg.data = tempGoal;
    pubGoal.publish(tempMsg);
}

void DmpVelocityController::computeGoalOffset(){
    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpVelocityController: Could not get state interface from hardware when checking velocities");
    }
    std::array<double, 7> currentPos;

    try {
        auto state_handle = state_interface->getHandle("panda_robot");

        currentPos = state_handle.getRobotState().q;

    } catch (const hardware_interface::HardwareInterfaceException& e) {
        ROS_ERROR_STREAM(
                    "DmpVelocityController: Exception getting state handle: " << e.what());
    }

    for (int iter=0; iter<dofs; iter++){
        ctBatch.g_offset[iter] = abs(dmpGoal[iter] - currentPos[iter]);
    }

}

void DmpVelocityController::saveDmpData(){
    //-------handles config file access----------------------
    std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
    Config config(datasetPath, basePackagePath);
    std::string saveQsPath = config.getConfBasePath() + std::string("dmpQsWithCt.txt");
    std::string robotQsPath = config.getConfBasePath() + std::string("robotQsWithCt.txt");
    ROS_INFO("Logging internal q(i) state of dmp to file: %s\n", saveQsPath.c_str());
    UTILS::writeTrajTimeToText(saveDmpQ, saveTime, saveQsPath);
    UTILS::writeTrajTimeToText(saveRobotQ, saveTime, robotQsPath);
}


}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpVelocityController,
                       controller_interface::ControllerBase)
