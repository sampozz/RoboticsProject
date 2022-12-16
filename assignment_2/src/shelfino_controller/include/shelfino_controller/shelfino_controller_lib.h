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
    double angular_velocity;
    double linear_velocity;

    ros::Publisher velocity_pub;
    ros::Subscriber odometry_sub;

    Coordinates current_position;
    double current_rotation;
    Coordinates odometry_position;
    double odometry_rotation;

    /**
     * Callback function, listen to /shelfino/odom topic and update odometry position and rotation
     */
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

    void send_velocity(double linear_vel, double angular_vel);

    void send_velocity(double linear_vel, double angular_vel, int n);

public:
    ShelfinoController(double linear_velocity, double angular_velocity, double loop_frequency);

    void shelfino_move_to(Coordinates &pos, double yaw);

    void shelfino_rotate(double angle);

    void shelfino_move_forward(double distance);
};

#endif