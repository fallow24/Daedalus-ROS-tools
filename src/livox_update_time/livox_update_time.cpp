#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

sensor_msgs::PointCloud2 msg;

void lidarMSGcallback(const sensor_msgs::PointCloud2::ConstPtr &m)  
{
    std_msgs::Header header;
    header.seq =  m->header.seq;
    header.frame_id = m->header.frame_id;

    // important part right here
    header.stamp = ros::Time::now(); 

    msg.header = header;
    msg.height = m->height;
    msg.width = m->width;
    msg.data = m->data;
    msg.fields = m->fields;
    msg.is_bigendian = m->is_bigendian;
    msg.is_dense = m->is_dense;
    msg.point_step = m->point_step;
    msg.row_step = m->row_step;   
}

/*Publishes a constant lidar CustomMsg for testing purposes*/
int main(int argc, char **argv)
{
  uint seq = 0;
  ros::init(argc, argv, "lidar_pub");

  ros::NodeHandle n;

  ros::Subscriber lidar_sub = n.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1000, lidarMSGcallback);
  ros::Publisher lidar_pub = n.advertise<sensor_msgs::PointCloud2>("/livox/lidar_t", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    lidar_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
