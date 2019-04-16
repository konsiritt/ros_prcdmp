
#include <debugDummy.h>


// notes: idea for problems:
//        - right now i am starting and stopping at the same time, initially no controller can be stopped though
//        -

#define UNDEFINED -666

debugDummy::debugDummy(ros::NodeHandle &node) {
  std::cout<<"debugDummy: in the constructor now"<<std::endl;
  nodeHandle = &node;

  int nBFs;
  double dt;
  std::vector<double> initialPosition, goalPosition, gainA, gainB;
  std::vector<std::vector<double>> weights;
  loadDmpData(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  //----------------------initialize dmp runtime object----------------------
  initDmpObjects(nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);

  initCouplingObject(dt, gainA, gainB);

  ROS_INFO("DmpCtVelocityController: setup DMP objects");

  msgBatch.samples.reserve((int) 1/tau/dt/30);

  pubBatch = nodeHandle->advertise<common_msgs::SamplesBatch>("/prcdmp/episodic_batch", 1000);

  //----------------------subscriber that handles changes to the dmp coupling term----------------------
  subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 100, &debugDummy::ctCallback, this);


  this->starting(ros::Time::now());
}

void debugDummy::starting(const ros::Time& /* time */) {
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

    elapsedTime = ros::Duration(0.0);
}

void debugDummy::update(const ros::Time& /* time */,
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

//    ROS_INFO("done with update");
    iterateRef++;
}

debugDummy::~debugDummy(){
}

void debugDummy::initializedCallback(const std_msgs::Bool::ConstPtr& msg)
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
      std::cout<<"debugDummy: response to switch service of dmp_velocity_controller is ok"<<std::endl;
    }
    else {
      std::cout<<"debugDummy: response to switch service of dmp_velocity_controller is NOT ok"<<std::endl;
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

void debugDummy::executedCallback(const std_msgs::Bool::ConstPtr& msg)
{
  if (!msg->data) {
    ros::Duration(1.5).sleep();

    ROS_INFO("switching controllers now to return robot into starting pose");
    if (!clientSwitch.call(srvInit)) {
      ROS_INFO("switch controller service call failed in executedCallback");
    }
    // debugging
    if (srvInit.response.ok){
      std::cout<<"debugDummy: response to switch service of dmpstart_velocity_controller is ok"<<std::endl;
    }
    else {
      std::cout<<"debugDummy: response to switch service of dmpstart_velocity_controller is NOT ok"<<std::endl;
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

  std::cout<< "in the main now"<<std::endl;
  ros::init(argc, argv, "debugDummy");

  std::cout<< "after ros::init"<<std::endl;

  // NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle n;

  std::cout<<"about to go into constructor"<<std::endl;

  // initializes Service dummy, using callbacks to topics to switch controllers
  debugDummy serviceMngr(n);

  std::cout<< "constructed the debug dummy object"<<std::endl;

  // loop frequency of 1 Hz
  ros::Rate loop_rate(1000);

  // ros::ok == False handles interruptions
  int count = 0;

  ros::Time currentTime;
  ros::Duration diff;
  while (ros::ok())
  {
    diff = ros::Time::now() - currentTime;
    currentTime = ros::Time::now();

    serviceMngr.update(currentTime,diff);

    // to add possible subscription when receiving a callback
    ros::spinOnce();

    //use the loop rate object to get the desired rate
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

bool debugDummy::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
                                          std::vector<double> &goalv, std::vector<std::vector<double> > &w,
                                          std::vector<double> &gainA, std::vector<double> &gainB) {
    //----------------------load DMP specific config data from files----------------------
    std::string datasetPath = "set4";

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

    refDmpTraj.reserve((int) timeSpan/dt);
    refDmpVel.reserve((int) timeSpan/dt);

    return true;
}

void debugDummy::initDmpObjects(int &nBFs, double &dt, std::vector<double> &initialPosition,
                                             std::vector<double> &goalPosition, std::vector<std::vector<double> > &weights,
                                             std::vector<double> &gainA, std::vector<double> &gainB) {
    //DiscreteDMP dmpTemp(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
    dmp = DiscreteDMP(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);;
    std::vector<std::vector<double>> dummY;
    dmp.rollout(refDmpTraj,refDmpVel,dummY,externalForce,tau, -1, 0);
    dmp.resettState();
}

void debugDummy::initCouplingObject (double &dt, std::vector<double> &gainA,
                                                  std::vector<double> &gainB){
    std::vector<double> initCoupling(dofs, 0.0);
    std::vector<double> goalCoupling(dofs, 0.0);
    scaleCoupling = 10; // i.e. 10 steps of coulingDmp per dmp step
    timeCouplingFinal = 0.003; // [ms] until final value is 99% reached
    couplingDmp = DiscreteDMP(dofs,dt/scaleCoupling,initCoupling,goalCoupling,gainA,gainB);

    //set current coupling term to zero
    couplingTerm = std::vector<double>(dofs,0.0);
}

void debugDummy::advanceCouplingTerm(){
    //taking intermediate steps in order to allow faster dynamic
    for (int i=0; i<scaleCoupling; ++i) {
        couplingDmp.simpleStep(externalForce, 1.0/timeCouplingFinal/scaleCoupling); // 1/timeCouplingFinal/scaleCoupling : final value should be achieved at timeCouplingFinal
    }
    couplingTerm = couplingDmp.getY();
    dmp.setCouplingTerm(couplingTerm);
}

void debugDummy::checkStoppingCondition(){
    //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities
    if (dmp.getTrajFinished()) {
        //publish the changed states to a topic so that the controller_manager can switch controllers
        if (!flagPubEx) {
            ROS_INFO("checkStoppingCondition: finished the target trajectory after time[s]: [%f]", elapsedTime.toSec());
            flagPubEx = true;
            pubBatch.publish(msgBatch);
            this->starting(ros::Time::now());
            std::cout<<"restarting the dmp now"<<std::endl;
        }
    }
}

void debugDummy::ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    std::cout<<"we are getting a callback"<<std::endl;
    if(!dmp.getTrajFinished() && elapsedTime.toSec()>0.0 && msgCoupling.msg_id != UNDEFINED){
        addCurrMessage();
    }
    msgCoupling = *msg;
    msgCoupling.header = msg->header;
    msgCoupling.data = msg->data;
    msgCoupling.msg_id = msg->msg_id;
    std::vector<double> temp(msgCoupling.data.begin(),msgCoupling.data.end());
    couplingDmp.setInitialPosition(couplingTerm);
    couplingDmp.setFinalPosition(temp);
}

void debugDummy::addCurrMessage(){
    ROS_INFO("adding a new batch to send back to Elie");
//    common_msgs::MDPSample tempMsg;
    tempMsg.ct = msgCoupling;
    tempMsg.reward = 0.0;
    tempMsg.mask = 0;
    boost::array<double,7> tempArray = {0.0};
    for (int i=0; i < 7; ++i) {
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
