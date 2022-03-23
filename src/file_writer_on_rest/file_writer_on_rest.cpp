#include <iostream>
#include <fstream>
#include <cassert>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>

// FIXME: Put pose and point cloud topics here:
#define POSE_TOPIC "/base_controller_node/odom"
#define PCL_TOPIC "/unit_sync/output"

// If in the last N_PAST_POSES ...
#define N_PAST_POSES 50
// ... the difference is always smaller than EPS_POSE_DIFF ...
#define EPS_POSE_DIFF 0.001
// ... a scan should be exported to PATH.
const char* PATH = "./"; 

// Internal variables
bool append_mode = true;
uint seq;
bool firstLidarCallback = false;
pcl::PCLPointCloud2 pcl_tmp;
pcl::PCLPointCloud2 pcl_export;
double _x, _y, _z, _rW, _rX, _rY, _rZ; // current pose
double x_, y_, z_, rW_, rX_, rY_, rZ_; // previous pose
int not_changed = 0;

/**
 * Converts a right-hand-side matrix into a 3DTK matrix
 * @param *inMatrix pointer to matrix (double[16])
 * @param *outMatrix pointer to matrix (double[16])
 * @param scale used for unit conversion, default 100.0 for Riegl
 */
inline void to3DTKMat(const double *inMatrix,
				  double *outMatrix, float scale = 100.0)
{
    outMatrix[0] = inMatrix[5];
    outMatrix[1] = -inMatrix[9];
    outMatrix[2] = -inMatrix[1];
    outMatrix[3] = -inMatrix[13];
    outMatrix[4] = -inMatrix[6];
    outMatrix[5] = inMatrix[10];
    outMatrix[6] = inMatrix[2];
    outMatrix[7] = inMatrix[14];
    outMatrix[8] = -inMatrix[4];
    outMatrix[9] = inMatrix[8];
    outMatrix[10] = inMatrix[0];
    outMatrix[11] = inMatrix[12];
    outMatrix[12] = -scale*inMatrix[7];
    outMatrix[13] = scale*inMatrix[11];
    outMatrix[14] = scale*inMatrix[3];
    outMatrix[15] = inMatrix[15];
}

static inline void Matrix4ToEuler(const double *alignxf,
                                  double *rPosTheta,
                                  double *rPos = 0)
{

  double _trX, _trY;

  // Calculate Y-axis angle
  if(alignxf[0] > 0.0) {
    rPosTheta[1] = asin(alignxf[8]);
  } else {
    rPosTheta[1] = M_PI - asin(alignxf[8]);
  }

  double  C    =  cos( rPosTheta[1] );
  if ( fabs( C ) > 0.005 )  {                 // Gimbal lock?
    _trX      =  alignxf[10] / C;             // No, so get X-axis angle
    _trY      =  -alignxf[9] / C;
    rPosTheta[0]  = atan2( _trY, _trX );
    _trX      =  alignxf[0] / C;              // Get Z-axis angle
    _trY      = -alignxf[4] / C;
    rPosTheta[2]  = atan2( _trY, _trX );
  } else {                                    // Gimbal lock has occurred
    rPosTheta[0] = 0.0;                       // Set X-axis angle to zero
    _trX      =  alignxf[5];  //1                // And calculate Z-axis angle
    _trY      =  alignxf[1];  //2
    rPosTheta[2]  = atan2( _trY, _trX );
  }

  rPosTheta[0] = rPosTheta[0];
  rPosTheta[1] = rPosTheta[1];
  rPosTheta[2] = rPosTheta[2];

  if (rPos != 0) {
    rPos[0] = alignxf[12];
    rPos[1] = alignxf[13];
    rPos[2] = alignxf[14];
  }
}

void poseMsgCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // Get current pose
  _x = msg->pose.pose.position.x;
  _y = msg->pose.pose.position.y;
  _z = msg->pose.pose.position.z;
  _rW = msg->pose.pose.orientation.w;
  _rX = msg->pose.pose.orientation.x;
  _rY = msg->pose.pose.orientation.y;
  _rZ = msg->pose.pose.orientation.z;
  
  // Check difference to last pose
  if ( 
    fabs(_x - x_) < EPS_POSE_DIFF &&
    fabs(_y - y_) < EPS_POSE_DIFF &&
    fabs(_z - z_) < EPS_POSE_DIFF &&
    fabs(_rW - rW_) < EPS_POSE_DIFF &&
    fabs(_rX - rX_) < EPS_POSE_DIFF &&
    fabs(_rY - rY_) < EPS_POSE_DIFF &&
    fabs(_rZ - rZ_) < EPS_POSE_DIFF
  ) { 
    append_mode = true;
    not_changed++;
  } else {
    //ROS_INFO("The robot moves to fast.");
    not_changed = 0;
    append_mode = false;
  }

  // If change is very small for N past poses, export Scan
  if (not_changed >= N_PAST_POSES) 
  {
    ROS_INFO("The robot has been very still. Exporting now.");
    not_changed = 0;
    
    // Transform PCL2 into appropriate, readable format.
    // PCL2 has data field, PCL has points[x, y, z] fields.
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud( new pcl::PointCloud<pcl::PointXYZI> );
    pcl::fromPCLPointCloud2( pcl_export, *temp_cloud);
    // Reset exported point cloud
    pcl_export = pcl_tmp;

    // Opening 3d file to write into
    std::ofstream file_3d;
    char* file_name = new char[50]();
    std::sprintf(file_name, "%sscan%03d.3d", PATH, seq);
    file_3d.open(file_name);

    // Writing the lidar data to the text file
    // This is a left handed coordinate system and we convert the values to cm
    for (int i = 0; i < temp_cloud->points.size(); ++i)
    {
        pcl::PointXYZI p = temp_cloud->points[i];
        
        // Convert to left handed
        file_3d <<  100.0 * p.y << " "
                << -100.0 * p.z << " "
                <<  100.0 * p.x << " "
                << temp_cloud->points[i].intensity << std::endl;

        // file_3d << -100.0 * p.y << " "
        //         <<  100.0 * p.z << " "
        //         <<  100.0 * p.x << " "
        //         << temp_cloud->points[i].intensity << std::endl;
    }
    file_3d.close();

    // Opening the pose file to write into
    std::ofstream file_pose;
    std::sprintf(file_name, "%sscan%03d.pose", PATH, seq);

    // Getting the current pose as homgeneous transformation (4x4)
    tf::Quaternion q(_rW, _rX, _rY, _rZ); 
    tf::Matrix3x3 m(q);
    const double in_matrix[16] ={m[0][0],m[0][1],m[0][2], _x,
                                 m[1][0],m[1][1],m[1][2], _y,
                                 m[2][0],m[2][1],m[2][2], _z,
				                         0      ,0      ,0      , 1};

    // Converting to left handed matrix
    double out_matrix[16], rPos[3], rPosTheta[16];
    to3DTKMat(in_matrix, out_matrix,1);
    Matrix4ToEuler(out_matrix,rPosTheta,rPos);

    // Extracting Position and Orientaion
    double x = 100.0*rPos[0];
    double y = 100.0*rPos[1];
    double z = 100.0*rPos[2];
    double roll  = 1.0*rPosTheta[0];
    double pitch = 1.0*rPosTheta[1];
    double yaw   = 1.0*rPosTheta[2];

    // Writing to the pose file
    // This is also a left hand coordinate system
    // - Thumb (x) to the right
    // - Pointy Finger (y) to  the top
    // - Middle Finger (z) into the room
    file_pose.open(file_name);
    file_pose << x << " " << y << " " << z << " " <<
                 roll * (180.0/M_PI) << " " <<
                 pitch * (180.0/M_PI) << " " <<
                 yaw * (180.0/M_PI) << " " << std::endl;
    file_pose.close();

    seq++;
    delete file_name;
  }

  // Save current pose for last pose in next iteration
  x_ = _x;
  y_ = _y;
  z_ = _z;
  rW_ = _rW;
  rX_ = _rX;
  rY_ = _rY;
  rZ_ = _rZ;
}

void lidarMsgCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  pcl_conversions::toPCL( *msg, pcl_tmp );
  if (append_mode) {
    pcl_export += pcl_tmp;
  } else {
    pcl_export = pcl_tmp;
  }
}

int main(int argc, char **argv)
{
    seq = 0;

    ros::init(argc, argv, "file_writer_on_rest");
    ros::NodeHandle n;
  
    ros::Subscriber pose_sub = n.subscribe(POSE_TOPIC, 10000, poseMsgCallback);
    ros::Subscriber lidar_sub = n.subscribe(PCL_TOPIC, 10000, lidarMsgCallback);

    ros::spin();
}
