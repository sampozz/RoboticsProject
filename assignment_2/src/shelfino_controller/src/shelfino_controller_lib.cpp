#include "shelfino_controller/shelfino_controller_lib.h"
#include "kinematics_lib/shelfino_kinematics.h"

using namespace std;

/* Public functions */

ShelfinoController::ShelfinoController(double linear_velocity, double angular_velocity, double loop_frequency) : loop_rate(loop_frequency)
{
    this->loop_frequency = loop_frequency;
    this->linear_velocity = linear_velocity;
    this->angular_velocity = angular_velocity;
    this->current_rotation = 0;
    this->odometry_rotation = 0;
    this->current_position << 2, 2, 0;

    // Publisher initialization
    velocity_pub = node.advertise<geometry_msgs::Twist>("/shelfino/velocity/command", 1000);

    // Subscriber initialization
    odometry_sub = node.subscribe("/shelfino/odom", 1, &ShelfinoController::odometry_callback, this);
}

void ShelfinoController::shelfino_move_to(Coordinates &pos, double yaw)
{
    // Compute the first rotation to make shelfino look towards the destination point
    double first_rot = -shelfino_trajectory(current_position, current_rotation, pos);
    ros::spinOnce(); // Update values from odometry

    double movement_duration = abs(first_rot / angular_velocity);
    double elapsed_time = 0;
    while (ros::ok())
    {
        // Select rotation direction and publish to topic
        first_rot > 0 ? send_velocity(0, angular_velocity) : send_velocity(0, -angular_velocity);

        if (elapsed_time > movement_duration)
            break;

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop rotation
    send_velocity(0, 0);
    ros::spinOnce();

    // Move forward and reach desired position
    double distance = sqrt(pow(pos(0) - current_position(0), 2) + pow(pos(1) - current_position(1), 2));
    movement_duration = distance / linear_velocity;
    
    Coordinates des_pos;
    double linear_res = 0, angular_res = 0; // Output of the Lyapunov control
    
    elapsed_time = 0;
    while (ros::ok())
    {
        // Compute Lyapunov line control
        des_pos << current_position(0) + (linear_velocity * cos(first_rot) * elapsed_time), 
            current_position(1) + (linear_velocity * sin(first_rot) * elapsed_time), 0;
        line_control(odometry_position, odometry_rotation, des_pos, first_rot, linear_velocity, 0.0, linear_res, angular_res);
        
        // Send data from line control
        send_velocity(linear_res, angular_res);

        if (elapsed_time > movement_duration)
            break;

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop movement
    send_velocity(0, 0);
    ros::spinOnce();

    // Rotate shelfino to match final rotation yaw
    double final_rot = norm_angle(first_rot - yaw);
    if (final_rot > M_PI)
        final_rot = -(2 * M_PI - final_rot);

    movement_duration = abs(final_rot / angular_velocity);
    elapsed_time = 0;
    while (ros::ok())
    {
        // Select rotation direction and publish to topic
        final_rot > 0 ? send_velocity(0, -angular_velocity) : send_velocity(0, angular_velocity);

        if (elapsed_time > movement_duration)
            break;

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop rotation
    send_velocity(0, 0);
    ros::spinOnce();

    // Update current position and rotation
    current_position = pos;
    current_rotation = yaw;
}

/* Private functions */

void ShelfinoController::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    geometry_msgs::Point p = msg->pose.pose.position;
    odometry_position << p.x, p.y, 0;

    geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    // We are interested on the rotation about z axis (yaw) only
    odometry_rotation = quaternion_to_yaw(q.x, q.y, q.z, q.w);
}

void ShelfinoController::send_velocity(double linear_vel, double angular_vel)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;

    velocity_pub.publish(msg);
}