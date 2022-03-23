#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

geometry_msgs::Quaternion current_orientation;
geometry_msgs::Point current_position;

void positionMsgCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  current_position.x = msg->x;
  current_position.y = msg->y;
  current_position.z = msg->z;
}


void orientationMsgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  current_orientation.x = msg->orientation.x;
  current_orientation.y = msg->orientation.y;
  current_orientation.z = msg->orientation.z;
  current_orientation.w = msg->orientation.w;
}

int main(int argc, char **argv)
{
  uint seq = 0;

  ros::init(argc, argv, "pose_merger");
  ros::NodeHandle n;

  ros::Subscriber position_sub = n.subscribe("position", 1000, positionMsgCallback);
  ros::Subscriber orientation_sub = n.subscribe("orientation", 1000, orientationMsgCallback);

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    std_msgs::Header header;
    geometry_msgs::Pose pose;
    pose.position = current_position;
    pose.orientation = current_orientation;

    header.seq = seq++;
    header.stamp = ros::Time::now();
    header.frame_id = "/imu";
    //TODO add frame_id

    geometry_msgs::PoseStamped msg;
    msg.header = header;
    msg.pose = pose;

    pose_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
