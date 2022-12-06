#ifndef __SHELFINO_CONTROLLER_H__
#define __SHELFINO_CONTROLLER_H__

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kinematics_lib/kinematics_types.h"
#include "kinematics_lib/shelfino_kinematics.h"
#include <Eigen/Dense>
#include <math.h>

class ShelfinoController
{
private:
    ros::NodeHandle node;
    ros::Rate loop_rate;

    double loop_frequency;

    ros::Publisher velocity_pub;
    ros::Subscriber odometry_sub;

    Coordinates current_position;
    RotationMatrix current_rotation;
    Coordinates odometry_position;
    RotationMatrix odometry_rotation;

    /**
     * Callback function, listen to /shelfino/odom topic and update odometry position and rotation
     */
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

    void send_velocity(Velocity &linear_vel, Velocity &angular_vel);

public:
    ShelfinoController(double loop_frequency);
};

#endif