#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <sstream>

/*Publishes a constant position for testing purposes*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_pub");

  ros::NodeHandle n;

  ros::Publisher position_pub = n.advertise<geometry_msgs::Point>("position", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Point msg;

    msg.x = 0.0;
    msg.y = 0.0;
    msg.z = 0.0;

    position_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
