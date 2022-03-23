#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>

const float RADIUS_LARGE_GEAR = 0.02; //m
const float RADIUS_SMALL_GEAR = 0.01; //m
const float ROTATION_CONVERSION_RATE = RADIUS_LARGE_GEAR/RADIUS_SMALL_GEAR;
const float HELIX_RADIUS = 0.103; //m == 10.3cm
const float CABLE_DIAMETER = 0.0076; //m == 7.6mm
const float HELIX_SLOPE = CABLE_DIAMETER/(2*M_PI*HELIX_RADIUS);

// Debugging and Correction Reasons:
const float LIN_FACTOR = 1; // 0.5 seams reasonable when rolling 22m of cable

geometry_msgs::Point current_position;

void jointstateMsgCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // We assume no swinging of the ball,
    // Hence only the height (z-axis) is changed
    // according to the Helix arc length formula
    current_position.x = 0;
    current_position.y = 0;
    current_position.z = LIN_FACTOR * (msg->position[0])/(ROTATION_CONVERSION_RATE) * (HELIX_RADIUS * sqrt(1+HELIX_SLOPE*HELIX_SLOPE));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "encoder_2_height");
    ros::NodeHandle n;

    ros::Rate loop_rate(1000);

    ros::Subscriber jointstate_sub = n.subscribe("joint_states", 1000, jointstateMsgCallback);
    ros::Publisher position_pub = n.advertise<geometry_msgs::Point>("position", 1000);

    while(ros::ok())
    {
        position_pub.publish(current_position);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
