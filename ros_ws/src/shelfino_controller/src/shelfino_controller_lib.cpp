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
    this->current_position << 0, 0, 0;

    // Publisher initialization
    velocity_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Subscriber initialization
    odometry_sub = node.subscribe("/shelfino2/odom", 100, &ShelfinoController::odometry_callback, this);
    detection_sub = node.subscribe("/shelfino/yolo/detections", 10, &ShelfinoController::detection_callback, this);
}

double ShelfinoController::move_to(const Coordinates &pos, double yaw)
{
    ROS_DEBUG("Moving Shelfino: initial position: %.2f %.2f %.2f, initial rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
    disable_vision = true;

    // Compute the first rotation to make shelfino look towards the destination point
    double first_rot = shelfino_trajectory_rotation(current_position, current_rotation, pos);
    rotate(first_rot);
    
    // Move forward and reach desired position
    double distance = sqrt(pow(pos(0) - current_position(0), 2) + pow(pos(1) - current_position(1), 2));
    
    move_forward(distance, true);

    if (yaw == 0) {
        disable_vision = false;
        ROS_DEBUG("Moving Shelfino: final position: %.2f %.2f %.2f, final rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
        return current_rotation; 
    }

    // Rotate shelfino to match final rotation yaw
    double final_rot = norm_angle(yaw - current_rotation);
    rotate(final_rot);
    
    ROS_DEBUG("Moving Shelfino: final position: %.2f %.2f %.2f, final rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
    disable_vision = false;
    return current_rotation;
}

double ShelfinoController::point_to(const Coordinates &pos)
{
    ROS_DEBUG("Rotating Shelfino: initial position: %.2f %.2f %.2f, initial rotation: %.2f", current_position(0), current_position(1), current_position(2), current_rotation); 
    
    // Compute the first rotation to make shelfino look towards the destination point
    double rot = shelfino_trajectory_rotation(current_position, current_rotation, pos);
    rotate(rot);

    return current_rotation;
}

double ShelfinoController::rotate(double angle)
{
    double movement_duration = abs(angle / angular_velocity);
    double elapsed_time = 0;
    block_detected = false;
    
    while (ros::ok())
    {
        // Select rotation direction and publish to topic
        angle > 0 ? send_velocity(0, angular_velocity) : send_velocity(0, -angular_velocity);
     
        if (elapsed_time > movement_duration || block_detected)
        {
            if (block_detected)
                ROS_DEBUG("Detected block during rotation, breaking.");
            break;
        }

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop rotation
    send_velocity(0, 0, 10);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    
    if (angle > 0)
        current_rotation = current_rotation + angular_velocity * elapsed_time;
    else 
        current_rotation = current_rotation - angular_velocity * elapsed_time;
    return current_rotation;
}

Coordinates ShelfinoController::move_forward(double distance, bool control)
{
    ROS_DEBUG("Moving Shelfino forward: initial position: %.2f %.2f %.2f", current_position(0), current_position(1), current_position(2)); 
    double movement_duration = abs(distance / linear_velocity);
    Coordinates des_pos;
    double elapsed_time = 0;
    double linear_res = 0, angular_res = 0; // Output of the Lyapunov control

    block_detected = false;

    while (ros::ok())
    {
        if (control)
        {
            // Compute Lyapunov line control
            des_pos << current_position(0) + (linear_velocity * cos(current_rotation) * elapsed_time), 
                current_position(1) + (linear_velocity * sin(current_rotation) * elapsed_time), 0;
            line_control(odometry_position, odometry_rotation, des_pos, current_rotation, linear_velocity, 0.0, linear_res, angular_res);

            // Publish to topic
            send_velocity(linear_res, angular_res);
        }
        else
        {
            // Publish to topic
            send_velocity(linear_velocity, 0);
        }

        if (elapsed_time > movement_duration || block_detected && control)
        {
            if (block_detected && control)
                ROS_DEBUG("Detected block during movement, breaking.");
            break;
        }

        loop_rate.sleep();
        ros::spinOnce();
        elapsed_time += 1.0 / loop_frequency;
    }
    // Stop movement
    send_velocity(0, 0, 10);
    ros::Duration(1.0).sleep();
    ros::spinOnce();
    current_position(0) = current_position(0) + elapsed_time * linear_velocity * cos(current_rotation);
    current_position(1) = current_position(1) + elapsed_time * linear_velocity * sin(current_rotation);
    ROS_DEBUG("Moving Shelfino forward: final position: %.2f %.2f %.2f", current_position(0), current_position(1), current_position(2)); 

    return current_position;
}

void ShelfinoController::reset_odometry(void)
{
    ros::spinOnce();
    odometry_position_0 = odometry_position;
    odometry_rotation_0 = odometry_rotation;

    if (abs(odometry_position_0(0) > 100) || abs(odometry_position_0(1)) > 100)
        ROS_WARN("Shelfino odometry broken");
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

void ShelfinoController::detection_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg) 
{
    if (msg->n > 0 && !disable_vision)
    {
        // Block detected
        if (!msg->bounding_boxes[0].is_blacklisted)
            block_detected = true;
    }
}

void ShelfinoController::send_velocity(double linear_vel, double angular_vel) const
{
    geometry_msgs::Twist msg;
    
    if (linear_vel > this->linear_velocity)
        linear_vel = this->linear_velocity;
    if (angular_vel > this->angular_velocity)
        angular_vel = this->angular_velocity;
    if (angular_vel < -this->angular_velocity)
        angular_vel = -this->angular_velocity;

    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;

    velocity_pub.publish(msg);
}

void ShelfinoController::send_velocity(double linear_vel, double angular_vel, int n) const
{
    for (int i = 0; i < n; i++)
    {
        send_velocity(linear_vel, angular_vel);
    }
}