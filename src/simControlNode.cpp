#include <simControlNode.h>

#include <cmath>

namespace prcdmp_node {

simControlNode::simControlNode(ros::NodeHandle& node_handle):
trajClient("/panda_arm_controller/follow_joint_trajectory", true){
    init(node_handle);
    starting(ros::Time::now());
}

bool simControlNode::init(ros::NodeHandle& node_handle) {

    nodeHandle = &node_handle;

    initROSCommunication();

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

    setupSampling();
    return true;
}

void simControlNode::starting(const ros::Time& /* time */) {
    ROS_INFO("simControlNode: starting()");
    // initialize the dmp trajectory (resetting the canonical sytem)
    dmp.resettState();
    //assume initialization
    flagPubEx = false;
    flagPubErr = false;
    refIter = 0;

    sampleDmpInit();

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

    actionGoal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    actionGoal.trajectory.points[0].positions = dmp.getY();
    trajClient.sendGoal(actionGoal);
    trajClient.waitForResult(ros::Duration(3.0));
    if (trajClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Robot initialized successfully");
        // send message that robot is initialized, so that mdp manager knows robot is good to go
        std_msgs::Bool msg;
        msg.data = false;
        pubInit.publish(msg);
    }
    else
    {
        ROS_INFO("Robot initialized unsuccessfull");
    }


    elapsedTime = ros::Duration(0.0);
}

bool simControlNode::update(const ros::Time& /* time */,
                                   const ros::Duration& period) {
    elapsedTime += period;

    std::vector<double> dq(7,0.0000001);

    dq = dmp.step(externalForce, tau);
    refIter++;

    if (logging) {
        std::vector <double> tempQ = dmp.getY();
        saveDmpQ.push_back(tempQ);
        std::copy(robotQ.begin(),robotQ.end(),tempQ.begin());
        saveRobotQ.push_back(tempQ);
        saveTime.push_back(elapsedTime.toSec());
    }

    bool done = checkStoppingCondition();

    commandRobot(dq);

    return done;
}

void simControlNode::stopping(const ros::Time& /*time*/) {
    // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
    // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
    // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    //if (ctBatch.samples.size() > 30){

    computeGoalOffset();
    pubBatch.publish(ctBatch);
    ROS_INFO("simControlNode: batch size published: #%d", ctBatch.samples.size());
    //}
    refIter = -1;

    if (logging){
        ROS_INFO("logging is on: dumping last dmp trajectory to file");
        saveDmpData();
    }
}

void simControlNode::initROSCommunication(){
    pubExec = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_exec", 1);
    pubError = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/error_occured", 1);
    pubBatch = nodeHandle->advertise<common_msgs::SamplesBatch>("/prcdmp/episodic_batch", 5);
    pubGoal = nodeHandle->advertise<std_msgs::Float64MultiArray>("/prcdmp/q_goal", 1);
    pubInit = nodeHandle->advertise<std_msgs::Bool>("/prcdmp/flag_notInit", 1);

    // subscriber that handles changes to the dmp coupling term: TODO: change to coupling term
    subCoupling = nodeHandle->subscribe("/coupling_term_estimator/coupling_term", 1, &simControlNode::ctCallback, this);
    // subscriber that handles changes to the smoothed dmp coupling term
    subCouplingSmoothed = nodeHandle->subscribe("/coupling_term_estimator/coupling_term/smoothed", 1, &simControlNode::ctSmoothedCallback, this);

    subFrankaStates = nodeHandle->subscribe("/joint_states", 1, &simControlNode::frankaStateCallback, this);

    subMdpReady = nodeHandle->subscribe("/mdp_manager/ready_for_dmp", 10, &simControlNode::mdpReadyCallback, this);

    if (!nodeHandle->getParam("/simControlNode/std_offset_q0", stdOffset)) {
      ROS_ERROR("simControlNode: Invalid or no std_offset_q0 parameter provided; provide e.g. std_offset_q0:=0.05");
    }

    if (!nodeHandle->getParam("/simControlNode/logging", logging)) {
      ROS_ERROR("simControlNode: Invalid or no logging parameter provided; provide e.g. logging:=true");
    }

    if (!nodeHandle->getParam("/simControlNode/seed", seed)) {
      ROS_ERROR("simControlNode: Invalid or no logging parameter provided; provide e.g. seed:=666");
    }    

    // follow joint trajectories using actions. these need a goal to be set
    actionGoal.trajectory.joint_names.push_back("panda_joint1");
    actionGoal.trajectory.joint_names.push_back("panda_joint2");
    actionGoal.trajectory.joint_names.push_back("panda_joint3");
    actionGoal.trajectory.joint_names.push_back("panda_joint4");
    actionGoal.trajectory.joint_names.push_back("panda_joint5");
    actionGoal.trajectory.joint_names.push_back("panda_joint6");
    actionGoal.trajectory.joint_names.push_back("panda_joint7");
    actionGoal.trajectory.points.resize(1);
    actionGoal.trajectory.points[0].positions.resize(7);
}


bool simControlNode::loadDmpData(int &nBFs, double &dt, std::vector<double> &y0v,
                                        std::vector<double> &goalv, std::vector<std::vector<double> > &w,
                                        std::vector<double> &gainA, std::vector<double> &gainB) {
    //----------------------load DMP specific config data from files----------------------
    if (!nodeHandle->getParam("/simControlNode/data_set", datasetPath)) {
        ROS_ERROR("simControlNode: Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
        return false;
    }
    //-------handles config file access----------------------
    std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/data/");
    Config config(datasetPath, basePackagePath);
    //------fill data from json to variables----------------------
    dofs = config.getDmpJson()["dofs"].asInt();
    std::cout<<"simControlNode: DOFs: "<<dofs<<std::endl;
    nBFs = config.getDmpJson()["n_basis"].asInt();
    dt = config.getDmpJson()["dt"].asDouble();
    double timeSpan = config.getDmpJson()["timespan"].asDouble();
    tau = 1.0/timeSpan;
    //------initialize arrays from config file----------------------
    moveJsonArrayToVec(config.getDmpJson()["q0"], y0v);
    dmpQ0 = y0v;
    moveJsonArrayToVec(config.getDmpJson()["goal"], goalv);
    moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
    moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);
    //------fill data from json to variables----------------------
    int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
    ROS_INFO("simControlNode: executing episode #%d",episodeNr);
    config.fillTrajectoryPath(episodeNr);
    if (episodeNr ==0) {
        UTILS::loadWeights(config.getInitialWPath(),w);
    }
    else {
        UTILS::loadWeights(config.getwPath(),w);
    }

    return true;
}

void simControlNode::initDmpObjects(int &nBFs, double &dt, std::vector<double> &initialPosition,
                                           std::vector<double> &goalPosition, std::vector<std::vector<double> > &weights,
                                           std::vector<double> &gainA, std::vector<double> &gainB) {
    //DiscreteDMP dmpTemp(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
    dmp = DiscreteDMP(dofs, nBFs, dt, initialPosition, goalPosition, weights, gainA, gainB);
}


bool simControlNode::checkStoppingCondition(){
    //TODO: find appropriate stopping behavior: e.g. (near) zero commanded velocities
    if (dmp.getTrajFinished()) {
        //publish the changed states to a topic so that the controller_manager can switch controllers
        if (!flagPubEx) {
            ROS_INFO("simControlNode: finished the target trajectory after time[s]: [%f]", elapsedTime.toSec());
            std_msgs::Bool msg;
            msg.data = false;
            pubExec.publish(msg);
            flagPubEx = true;
        }
        return true;
    }
    return false;
}

void simControlNode::commandRobot(const std::vector<double> &dq){
    double omega = 0.0;
    int it = 0;
    for (int iter = 0; iter<dofs; iter++) {
        omega = dq[it];
        it++;
    }

    //trajClient.waitForServer();
    actionGoal.trajectory.points[0].time_from_start = ros::Duration(0.001);
    actionGoal.trajectory.points[0].positions = dmp.getY();
    trajClient.sendGoal(actionGoal);
    //trajClient.waitForResult(ros::Duration(1.0));
}

//TODO: adapt to react to a change of the coupling term as a topic
void simControlNode::ctCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    // do not listen, when the controller is done (flagPubEx) or when it is not actie (refIter =-1)
    if (!flagPubEx && refIter >= 0) {

        if (refIter > 0 && !firstCB)
        {
            std::vector<double> currQ = dmp.getY();
            for (int iter=0; iter<dofs; iter++)
            {
                if (refIter-1 > refQ.size())
                {
                    ROS_INFO("simControlNode: we are accessing the reference trajectory out of bounds");
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

void simControlNode::ctSmoothedCallback(const common_msgs::CouplingTerm::ConstPtr& msg) {
    std::vector<double> coupling(msg->data.begin(),msg->data.end());
    dmp.setCouplingTerm(coupling);
}

void simControlNode::frankaStateCallback(const sensor_msgs::JointState& msg) {
    //currentRobotMode = msg->robot_mode;
    for (size_t iter = 0; iter < dofs; iter++) {
        robotQ[iter] = msg.position[iter+2];
    }
}

void simControlNode::mdpReadyCallback (const std_msgs::Bool::ConstPtr& msg) {
    mdpReady = msg->data;
}

bool simControlNode::isValidVelocity(std::vector<double> velocitiesToApply) {
    // check for initial joint positions of the robot
    //TODO: currently no check for simulation

    return true;
}

void simControlNode::setupSampling(){
    unsigned tempSeed = (unsigned) seed;
    generator = std::default_random_engine (tempSeed);
    distribution = std::normal_distribution<double>(meanOffset,stdOffset);
}

std::vector <double> simControlNode::getRandomVectorOffset(){
    std::vector<double> randomVector(7,0.0);
    for (int iterate=0; iterate<dofs; iterate++){
        randomVector[iterate] = distribution(generator);
    }
    return randomVector;
}

std::vector<double> simControlNode::addVectors(const std::vector<double> &element1, const std::vector<double> &element2){
    assert(element1.size() == element2.size());
    std::vector<double> returnVector(element1.size(),0.0);
    for (int iterator=0; iterator < element1.size(); iterator++) {
        returnVector[iterator] = element1[iterator] + element2[iterator];
    }
    return returnVector;
}

void simControlNode::sampleDmpInit(){
    // adapt the dmp to the initial joint positions of the robot
    std::vector <double> tempQ = sampleVector(dmpQ0);

    dmp.setInitialPosition(tempQ);
    dmp.resettState();
}

void simControlNode::sampleGoalQ(){
    // adapt the dmp to the initial joint positions of the robot
    std::vector<double> dmpGoalV(dmpGoal.begin(), dmpGoal.end());
    std::vector<double> qGoalWithOffsetV = sampleVector(dmpGoalV);
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

std::vector<double> simControlNode::sampleVector(const std::vector<double> inputVector) {
    std::vector<double> offsetV = getRandomVectorOffset();
    std::vector<double> inWithOffsetV = addVectors(inputVector,offsetV);
    return inWithOffsetV;
}

void simControlNode::computeGoalOffset(){

    for (int iter=0; iter<dofs; iter++){
        ctBatch.g_offset[iter] = abs(dmpGoal[iter] - robotQ[iter]);
    }

}

void simControlNode::saveDmpData(){
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

int main(int argc, char** argv) {
  ros::init(argc, argv, "sim_control_node");
  ros::NodeHandle public_node_handle;
  ros::NodeHandle node_handle("~");  

  ros::Rate loop_rate(1000);

  // Start background threads for message handling
  ros::AsyncSpinner spinner(4);
  spinner.start();

  prcdmp_node::simControlNode controlNodeObj(node_handle);

  bool done = false;
  bool mdpReady = true;
  ros::Time last_time = ros::Time::now();
  while (ros::ok()) {
      ros::Duration(0.5).sleep();
      // check if the mdp manager is ready before allowing rolling out of dmp
      mdpReady = controlNodeObj.getMdpReady();
      if (mdpReady) {
          done = false;
      }

      while (!done && mdpReady) {
          // Run control loop. Will exit if the controller is switched.
          ros::Time now = ros::Time::now();
          done = controlNodeObj.update(now, now - last_time);
          last_time = now;

          if (!ros::ok()) {
              return 0;
          }
          if (done) {
              // when the dmp is done once, stop the dmp trajectory execution and restart again
              controlNodeObj.stopping(ros::Time::now());
              controlNodeObj.starting(ros::Time::now());
          }
          ros::spinOnce();

          loop_rate.sleep();
      }
  }

  return 0;
}




