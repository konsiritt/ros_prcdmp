#include "ros/ros.h"
#include "std_msgs/Bool.h"


int main(int argc, char **argv)
/*
 * 1. Initialize the ROS system
 * 2. Advertize that we are publishing on the chatter topic to the master
 * 3. Loop while publishing messages to chatter 10 times a second
 */
{
  ros::init(argc, argv, "talkerDummy");

  // NodeHandle is the main access point to communications with the ROS system.
  ros::NodeHandle n;

  // publishing string on the topic "chatter" with a buffer of 1000 msgs
  ros::Publisher chatter_pub = n.advertise<std_msgs::Bool>("/prcdmp/dmp_exec", 10);

  std_msgs::Bool msg;

  // loop frequency of 1 Hz
  ros::Rate loop_rate(1);

  // ros::ok == False handles interruptions
  int count = 0;
  while (ros::ok())
  {

    msg.data = true;

    // ROS version of cout
    //ROS_INFO("%s", msg.data.c_str());

    // broadcast the message
    chatter_pub.publish(msg);

    // to add possible subscription when receiving a callback
    ros::spinOnce();

    //use the loop rate object to get the desired rate
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
