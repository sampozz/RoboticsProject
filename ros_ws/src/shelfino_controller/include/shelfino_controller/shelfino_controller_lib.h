/** 
* @file shelfino_controller_lib.h 
* @brief Header file for the Shelfino Controller class and library
*
* @author Samuele Pozzani
*
* @date 18/12/2022
*/

#ifndef __SHELFINO_CONTROLLER_H__
#define __SHELFINO_CONTROLLER_H__

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kinematics_lib/kinematics_types.h"
#include "kinematics_lib/shelfino_kinematics.h"
#include "robotic_vision/BoundingBoxes.h"
#include <Eigen/Dense>
#include <math.h>

/**
 * @brief The Shelfino Controller class implements the high level functions for the movement and control of Shelfino
 * @class ShelfinoController
 */
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
    ros::Subscriber detection_sub;

    Coordinates current_position;
    double current_rotation;
    Coordinates odometry_position, odometry_position_0;
    double odometry_rotation, odometry_rotation_0;

    bool block_detected;
    bool disable_vision;

    /**
     * Callback function, listen to /shelfino/odom topic and update odometry position and rotation
     * 
     * @param msg The message received on the topic
     */
    void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);

    /**
     * Callback function, listen to /yolov5/detections topic and check for detected block
     * 
     * @param msg The message received on the topic
     */
    void detection_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg);

    /**
     * Send velocity values to /shelfino/velocity/command topic
     * 
     * @param linear_vel The valocity value for the x axis
     * @param angular_vel the velocity value for the z axis rotation
     */
    void send_velocity(double linear_vel, double angular_vel) const;

    /**
     * Send velocity values to /shelfino/velocity/command topic n times
     * 
     * @param linear_vel The valocity value for the x axis
     * @param angular_vel the velocity value for the z axis rotation
     * @param n The number of messages to send to topic
     */
    void send_velocity(double linear_vel, double angular_vel, int n) const;

public:
    /**
     * Constructor.
     * Create the shelfino controller, assign initial values and initialize publishers and subscribers
     * 
     * @param linear_velocity The linear velocity value for Shelfino (which is constant) 
     * @param angular_velocity The angular velocity value for Shelfino (which is constant) 
     * @param loop_frequency The default frequency used to compute trajectories and send messages to topics
     */
    ShelfinoController(double linear_velocity, double angular_velocity, double loop_frequency);

    /**
     * Move Shelfino from its current position to the desired final position and rotation.
     * This function executes the following steps:
     * 1. Compute the initial rotation angle to make Shelfino look towards the final point and rotate
     * 2. Compute the linear distance needed to reach the final point and move forwards using the Lyapunov control
     * 3. If yaw != 0, rotate to reach the desired final rotation
     * 
     * @param pos The desired final position
     * @param yaw The desired final rotation
     * @return Final rotation of shelfino
     */
    double move_to(const Coordinates &pos, double yaw);

    /**
     * Rotate Shelfino from its current rotation to look towards the desired position.
     * The rotation may be interrupted if a block is detected on the vision topic.
     * 
     * @param pos The desired final position
     * @return The final rotation
     */
    double point_to(const Coordinates &pos);

    /**
     * Rotate shelfino of the desired angle
     * The rotation may be interrupted if a block is detected on the vision topic.
     * 
     * @param angle The desired rotation
     * @return The final rotation of shelfino
     */
    double rotate(double angle);

    /**
     * Move shelfino forwards of the desired distance using lyapunov control.
     * Shelfino breaks if a block is detected during motion, only if control is enabled.
     * 
     * @param distance The distance that shelfino should travel
     * @param control Use Lyapunov control if True. Break if block detected.
     * @return Final position of Shelfino
    */
    Coordinates move_forward(double distance, bool control);

    /**
     * Reset the odometry values by setting the current position as origin
     */
    void reset_odometry(void);
};

#endif