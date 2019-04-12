#include <dmp_ct_velocity_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace prcdmp_node {

#define UNDEFINED -666

bool DmpCtVelocityController::init(hardware_interface::RobotHW* robot_hardware,
                                   ros::NodeHandle& node_handle) {
    robotHardware = robot_hardware;
    nodeHandle = &node_handle;

    if (!checkRobotSetup()) {return false;}

    //----------------------load DMP specific config data from files----------------------
    int nBFs;
    double dt;
    std::vector<double> initialPosition, goalPosition, gainA, gainB;
    std::vector<std::vector<double>> weights ;
    loadDmpData(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

    //----------------------initialize dmp runtime object----------------------
    initDmpObjects(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

    initCouplingObject(dt, gainA, gainB);

    ROS_INFO("DmpCtVelocityController: setup DMP objects");

    initROSCommunication();

    msgBatch.samples.reserve(1/tau/dt/30);

    return true;
}


void DmpCtVelocityController::starting(const ros::Time& /* time */) {
    ROS_INFO("DmpCtVelocityController: starting");
    // initialize the dmp trajectory (resetting the canonical sytem)
    dmp.resettState();
    couplingDmp.resettState();

    //reset iterator for reference joint positions
    iterateRef = 0;

    //reset the message batch for reward information
    msgBatch.samples.clear();

    //assume initialization
    flagPubEx = false;

    //set current coupling term to zero
    couplingTerm = std::vector<double>(dofs,0.0);

    auto state_interface = robotHardware->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR("DmpCtVelocityController: Could not get state interface from hardware when starting the controller");
    }

    elapsedTime = ros::Duration(0.0);
}

void DmpCtVelocityController::update(const ros::Time& /* time */,
                                     const ros::Duration& period) {
    elapsedTime += period;

//    ROS_INFO("advancing the coupling term");
    advanceCouplingTerm();

    //advance the actual dmp
//    ROS_INFO("advancing the dmp");
    std::vector<double> dq = dmp.step(externalForce, tau);
    qDmp = dmp.getY();

//    ROS_INFO("checking stopping conditions");
    checkStoppingCondition();

//    ROS_INFO("commanding on the robot");
    commandRobot(dq);

//    ROS_INFO("done with update");
    iterateRef++;
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
    if(!dmp.getTrajFinished() && elapsedTime.toSec()>0.0 && msgCoupling.msg_id != UNDEFINED){
        addCurrMessage();
    }

    // reset counter that counts control cycles per ct
    msgCoupling = *msg;
    msgCoupling.header = msg->header;
    msgCoupling.data = msg->data;
    msgCoupling.msg_id = msg->msg_id;
    std::vector<double> temp(msgCoupling.data.begin(),msgCoupling.data.end());
    couplingDmp.setInitialPosition(couplingTerm);
    couplingDmp.setFinalPosition(temp);
}

void DmpCtVelocityController::addCurrMessage(){
    common_msgs::MDPSample tempMsg;
    tempMsg.ct = msgCoupling;
    tempMsg.reward = 0.0;
    tempMsg.mask = 0;
    boost::array<double,7> tempArray = {0.0};
    for (int i=0; i < qDmp.size(); ++i) {
        tempArray[i] = refDmpTraj[iterateRef][i] - qDmp[i];
        if (tempArray[i]>10.0 || tempArray[i]<-10.0) {
            std::cout<<"refDmpTraj[iterateRef][i]: "<<refDmpTraj[iterateRef][i]<<std::endl;
            std::cout<<"qDmp[i]: "<<qDmp[i]<<std::endl;
            std::cout<<"tempArray[i] :"<<tempArray[i]<<std::endl;
        }
    }

    tempMsg.q_offset = tempArray;
    msgBatch.samples.push_back(tempMsg);
}

void DmpCtVelocityController::initROSCommunication(){
    //----------------------initialize the publishing nodes----------------------
    pubExec = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_exec", 1000);
    pubBatch = nodeHandle->advertise<common_msgs::SamplesBatch>("/prcdmp/episodic_batch", 1000);

    //----------------------subscriber that handles changes to the dmp coupling term----------------------
    subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 100, &DmpCtVelocityController::ctCallback, this);

    msgCoupling.msg_id = UNDEFINED;
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

    if (!nodeHandle->getParam("/franka_control/robot_ip", robotIp)) {
        ROS_ERROR("Invalid or no robot_ip parameter provided");
        return false;
    }
    return true;
}

bool DmpCtVelocityController::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
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

    refDmpTraj.reserve(int(timeSpan/dt));
    refDmpVel.reserve(int(timeSpan/dt));

    return true;
}

void DmpCtVelocityController::initDmpObjects(int &nBFs, double &dt, std::vector<double> &initialPosition,
                                             std::vector<double> &goalPosition, std::vector<std::vector<double> > &weights,
                                             std::vector<double> &gainA, std::vector<double> &gainB) {
    //DiscreteDMP dmpTemp(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
    dmp = DiscreteDMP(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);;
    std::vector<std::vector<double>> dummY;
    dmp.rollout(refDmpTraj,refDmpVel,dummY,externalForce,tau, -1, 0);
    dmp.resettState();
}

void DmpCtVelocityController::initCouplingObject (double &dt, std::vector<double> &gainA,
                                                  std::vector<double> &gainB){
    std::vector<double> initCoupling(dofs, 0.0);
    std::vector<double> goalCoupling(dofs, 0.0);
    scaleCoupling = 10; // i.e. 10 steps of coulingDmp per dmp step
    timeCouplingFinal = 0.003; // [ms] until final value is 99% reached
    couplingDmp = DiscreteDMP(dofs,dt/scaleCoupling,initCoupling,goalCoupling,gainA,gainB);

    //set current coupling term to zero
    couplingTerm = std::vector<double>(dofs,0.0);
}

void DmpCtVelocityController::advanceCouplingTerm(){
    //taking intermediate steps in order to allow faster dynamic
    for (int i=0; i<scaleCoupling; ++i) {
        couplingDmp.simpleStep(externalForce, 1/timeCouplingFinal/scaleCoupling); // 1/timeCouplingFinal/scaleCoupling : final value should be achieved at timeCouplingFinal
    }
    couplingTerm = couplingDmp.getY();
    dmp.setCouplingTerm(couplingTerm);
}

void DmpCtVelocityController::checkStoppingCondition(){
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

void DmpCtVelocityController::commandRobot(const std::vector<double> &dq){
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
}


}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpCtVelocityController,
                       controller_interface::ControllerBase)
