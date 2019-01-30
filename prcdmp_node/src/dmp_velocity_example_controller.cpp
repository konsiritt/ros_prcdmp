// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <dmp_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace prcdmp_node {

bool DmpVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "DmpVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("DmpVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("DmpVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "DmpVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("DmpVelocityExampleController: Could not get state interface from hardware");
    return false;
  }


  std::string datasetPath;
  if (!node_handle.getParam("data_set", datasetPath)) {
    ROS_ERROR("Invalid or no data_set parameter provided; provide e.g. data_set:=set1");
    return 1;
  }

  /// load DMP specific config data from files

  // handles config file access
  std::string basePackagePath = ros::package::getPath("prcdmp_node") + std::string("/");
  std::cout<<"this is the package base path: "<<basePackagePath<<std::endl;
  Config config(datasetPath, basePackagePath);
  std::cout<<"config file has been created"<<std::endl;

  //fill data from json to variables
  int dofs = config.getDmpJson()["dofs"].asInt();
  std::cout<<"DOFs: "<<dofs<<std::endl;
  int nBFs = config.getDmpJson()["n_basis"].asInt();
  double dt = config.getDmpJson()["dt"].asDouble();
  double timeSpan = config.getDmpJson()["timespan"].asDouble();
  tau = 1.0/timeSpan;

  // initialize arrays from config file
  std::array<double,7> q0;
  std::array<double,7> goal;
  std::vector<double> gainA, gainB;
  moveJsonArrayToVec(config.getDmpJson()["q0"], q0);
  moveJsonArrayToVec(config.getDmpJson()["goal"], goal);
  moveJsonArrayToVec(config.getDmpJson()["gain_a"], gainA);
  moveJsonArrayToVec(config.getDmpJson()["gain_b"], gainB);

  //fill data from json to variables
  std::string robotIp = config.getDataJson()["robot_ip"].asString();

  int episodeNr = config.getDataJson()["current_episode"].asInt()-1;
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

  // initialize dmp object
  DiscreteDMP dmpTemp(dofs, nBFs, dt, y0v, goalv, w, gainA, gainB);
  dmp = dmpTemp;

  double timesteps = dmp.getTimesteps();
  std::cout<<"amount of timesteps for current dmp: "<<timesteps<<std::endl;

  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("Invalid or no robot_ip parameter provided");
    return 1;
  }
  if (robotIp != robot_ip) {
    ROS_ERROR("JSON config file specifies other robot ip than function argument");
  }

  /// check if robot is in desired initial pose, and move there if not
  try {
    auto state_handle = state_interface->getHandle("panda_robot");

    for (size_t i = 0; i < q0.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q0[i]) > 0.1) {
        ROS_WARN(
            "DmpVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Trying to move the robot into its initial position now");
        try {
          std::array<double,13> load = {0,0,0,0,0,0,0,0,0,0,0,0,0}; // std::array is another way to define an array

          movePointToPoint(&robot_ip[0u],q0,0.1,load);
          return true;
        }
        catch (const franka::ControlException& e)
        {
             std::cout << e.what() << std::endl;
             return false;
         }
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "DmpVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}


void DmpVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}

void DmpVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  ros::Duration time_max(8.0);

  dmp.step(externalForce, tau);
  std::vector<double> dq = dmp.getDY();

  double omega = 0.0; 
  int it = 0;
  for (auto joint_handle : velocity_joint_handles_) {
    omega = dq[it];
    joint_handle.setCommand(omega);
    it++;
  }
}

void DmpVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void DmpVelocityExampleController::movePointToPoint(char* ip,std::array<double,7> qGoal,double speed, std::array<double,13> load)
{
    try
    {
        franka::Robot robot(ip); // initialization of the robot object passing the robot IP to its constructor function

        //set default behavior:
        robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        //robot.setFilters(1000, 1000, 1000, 1000, 1000);
        // Set external load for the manipulator ========================================================
        robot.setLoad(load[0],{load[1],load[2],load[3]},{load[4],load[5],load[6],load[7],load[8],load[9],load[10],load[11],load[12]});
        // ==============================================================================================

        double speed_factor = speed;

        // Set collision behavior.
        robot.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

        MotionGenerator motion_generator(speed_factor, qGoal);
        robot.control(motion_generator);
        std::cout << "Motion finished" << std::endl;

    }
    catch (const franka::Exception& e)
    {
        std::cout << e.what() << std::endl;
        return;
    }
}

}  // namespace prcdmp_node

PLUGINLIB_EXPORT_CLASS(prcdmp_node::DmpVelocityExampleController,
                       controller_interface::ControllerBase)
