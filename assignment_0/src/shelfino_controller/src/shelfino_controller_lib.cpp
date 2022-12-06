#include "shelfino_controller_lib/shelfino_controller.h"

/* Public functions */

ShelfinoController::ShelfinoController(double loop_frequency) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;

    // Publisher initialization
    velocity_pub = node.advertise<geometry_msgs::Twist>("/shelfino/velocity/command", 1000);

    // Subscriber initialization
    odometry_sub = node.subscribe("/shelfino/odometry", 1, &ShelfinoController::odometry_callback, this);
}

/* Private functions */

void ShelfinoController::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::Point p = msg->pose.pose.position;
    odometry_position << p.x, p.y, p.z;

    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    quaternion_to_rot(q.x, q.y, q.z, q.w, odometry_rotation);
}

void ShelfinoController::send_velocity(Velocity &linear_vel, Velocity &angular_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_vel(0);
    msg.linear.y = linear_vel(1);
    msg.linear.z = linear_vel(2);
    msg.angular.x = angular_vel(0);
    msg.angular.y = angular_vel(1);
    msg.angular.z = angular_vel(2);

    velocity_pub.publish(msg);
}