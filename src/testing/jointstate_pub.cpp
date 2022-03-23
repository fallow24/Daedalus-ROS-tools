#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>


/*Publishes a constantly varying joint state for testing purposes*/
int main(int argc, char **argv)
{
  uint seq = 0;
  ros::init(argc, argv, "orientation_pub");
  ros::NodeHandle n;

  ros::Publisher jointstate_pub = n.advertise<sensor_msgs::JointState>("joint_state", 1000);
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    std_msgs::Header header;
    sensor_msgs::JointState msg;

    header.seq =  seq++;
    header.stamp = ros::Time::now();
    msg.header = header;
    msg.name = {"Test"};

    msg.position = {2*M_PI/10000 * seq};

    jointstate_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
