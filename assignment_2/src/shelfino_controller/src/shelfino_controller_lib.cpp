#include "shelfino_controller/shelfino_controller_lib.h"
#include "kinematics_lib/shelfino_kinematics.h"
#include <unistd.h>
using namespace std;

/* Public functions */

ShelfinoController::ShelfinoController(double linear_velocity, double angular_velocity, double loop_frequency) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
    this->linear_velocity = linear_velocity;
    this->angular_velocity = angular_velocity;
    this->current_rotation = 0;
    this->odometry_rotation = 0;
    this->current_position << 0, 0, 0;

    // Publisher initialization
    velocity_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    // Subscriber initialization
    odometry_sub = node.subscribe("/shelfino2/odom", 100, &ShelfinoController::odometry_callback, this);
}

void ShelfinoController::shelfino_move_to(Coordinates &pos, double yaw)
{
    ROS_INFO("Moving Shelfino: initial position: %.2f %.2f %.2f, initial rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
    
    // Compute the first rotation to make shelfino look towards the destination point
    double first_rot = shelfino_trajectory(current_position, current_rotation, pos);
    shelfino_rotate(first_rot);
    
    // Move forward and reach desired position
    double distance = sqrt(pow(pos(0) - current_position(0), 2) + pow(pos(1) - current_position(1), 2));
    shelfino_move_forward(distance);

    if (yaw == 0) {
        ROS_INFO("Moving Shelfino: final position: %.2f %.2f %.2f, final rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
        return; 
    }

    // Rotate shelfino to match final rotation yaw
    double final_rot = norm_angle(first_rot - yaw);
    if (final_rot > M_PI)
        final_rot = -(2 * M_PI - final_rot);
    shelfino_rotate(final_rot);
}

void ShelfinoController::shelfino_rotate(double angle)
{
    double movement_duration = abs(angle / angular_velocity);
    double elapsed_time = 0;
    while (ros::ok())
    {
        // Select rotation direction and publish to topic
        angle > 0 ? send_velocity(0, angular_velocity) : send_velocity(0, -angular_velocity);
     
        if (elapsed_time > movement_duration)
            break;

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop rotation
    send_velocity(0, 0, 10);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    current_rotation = ((current_rotation + angle) + odometry_rotation) / 2;
}

void ShelfinoController::shelfino_move_forward(double distance)
{
    double movement_duration = abs(distance / linear_velocity);
    Coordinates des_pos;
    double elapsed_time = 0;
    double linear_res = 0, angular_res = 0; // Output of the Lyapunov control

    while (ros::ok())
    {
        // Compute Lyapunov line control
        des_pos << current_position(0) + (linear_velocity * cos(current_rotation) * elapsed_time), 
            current_position(1) + (linear_velocity * sin(current_rotation) * elapsed_time), 0;
        line_control(odometry_position, odometry_rotation, des_pos, current_rotation, linear_velocity, 0.0, linear_res, angular_res);

        // Publish to topic
        send_velocity(linear_res, angular_res);

        if (elapsed_time > movement_duration)
            break;

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop movement
    send_velocity(0, 0, 10);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    current_position(0) = (current_position(0) + distance * cos(current_rotation) + odometry_position(0)) / 2;
    current_position(1) = (current_position(1) + distance * sin(current_rotation) + odometry_position(1)) / 2;
}

void ShelfinoController::reset_odometry(void)
{
    ros::spinOnce();
    odometry_position_0 = odometry_position;
    odometry_rotation_0 = odometry_rotation;
}


/* Private functions */

void ShelfinoController::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::Point p = msg->pose.pose.position;
    odometry_position << p.x, p.y, 0;
    odometry_position -= odometry_position_0;

    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    // We are interested in the rotation about z axis (yaw) only
    odometry_rotation = quaternion_to_yaw(q.x, q.y, q.z, q.w);
    odometry_rotation -= odometry_rotation_0;
}

void ShelfinoController::send_velocity(double linear_vel, double angular_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;

    velocity_pub.publish(msg);
}

void ShelfinoController::send_velocity(double linear_vel, double angular_vel, int n)
{
    for (int i = 0; i < n; i++)
    {
        send_velocity(linear_vel, angular_vel);
    }
}