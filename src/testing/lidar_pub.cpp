#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

/*Publishes a constant lidar CustomMsg for testing purposes*/
int main(int argc, char **argv)
{
  uint seq = 0;
  ros::init(argc, argv, "lidar_pub");

  ros::NodeHandle n;

  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::PointCloud2 msg;
    std_msgs::Header header;
    header.seq =  seq++;
    header.stamp = ros::Time::now();

    msg.header = header;
    msg.height = 1;
    msg.width = 1;
    sensor_msgs::PointField field;
    field.name = "Test Field";
    field.offset = 1;
    field.datatype = 1; //INT8
    field.count = 1;
    msg.fields = {field};
    msg.is_bigendian = true;
    msg.point_step = 1;
    msg.row_step = 1;
    msg.data = {1,1,1};
    msg.is_dense = true;

    lidar_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
