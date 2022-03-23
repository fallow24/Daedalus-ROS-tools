#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <sstream>

/*Publishes a constant position for testing purposes*/
int main(int argc, char **argv)
{
  uint seq = 0;

  ros::init(argc, argv, "orientation_pub");

  ros::NodeHandle n;

  ros::Publisher orientation_pub = n.advertise<sensor_msgs::Imu>("orientation", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Quaternion orientation;

    //180 deg rotation about y-axis
    orientation.x = 0.0;
    orientation.y = 1.0;
    orientation.z = 0.0;
    orientation.w = 0.0;

    geometry_msgs::Vector3 angular_velocity;
    angular_velocity.x = 0;
    angular_velocity.y = 0;
    angular_velocity.z = 0;

    geometry_msgs::Vector3 linear_acceleration;
    linear_acceleration.x = 0;
    linear_acceleration.y = 0;
    linear_acceleration.z = 0;

    std_msgs::Header header;
    header.seq =  seq++;
    header.stamp = ros::Time::now();

    sensor_msgs::Imu msg;
    msg.header = header;
    msg.orientation = orientation;
    msg.orientation_covariance  = {0,0,0,0,0,0,0,0,0};
    msg.angular_velocity = angular_velocity;
    msg.angular_velocity_covariance = {0,0,0,0,0,0,0,0,0};
    msg.linear_acceleration = linear_acceleration;
    msg.linear_acceleration_covariance = {0,0,0,0,0,0,0,0,0};

    orientation_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
