#include "ros/ros.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/PointTo.h"
#include "shelfino_controller/MoveForward.h"
#include "std_srvs/SetBool.h"
#include <signal.h>

ShelfinoController *controller_ptr = nullptr;

std_srvs::SetBool shelfino_power_srv;
ros::ServiceClient shelfino_power;

bool move_to(shelfino_controller::MoveTo::Request &req, shelfino_controller::MoveTo::Response &res)
{
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;
    
    controller_ptr->shelfino_move_to(pos, req.rot);
    return true;
}

bool rotate(shelfino_controller::Rotate::Request &req, shelfino_controller::Rotate::Response &res)
{    
    double rot = controller_ptr->shelfino_rotate(req.angle);
    res.rot = rot;
    return true;
}

bool point_to(shelfino_controller::PointTo::Request &req, shelfino_controller::PointTo::Response &res)
{    
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;

    double angle = controller_ptr->shelfino_point_to(pos);
    res.rot = angle;
    return true;
}

bool move_forward(shelfino_controller::MoveForward::Request &req, shelfino_controller::MoveForward::Response &res)
{    
    Coordinates new_position = controller_ptr->shelfino_move_forward(req.distance, true);
    res.pos.x = new_position(0);
    res.pos.y = new_position(1);
    return true;
}

void handler(int sig)
{
    // Engines power off on CTRL+C
    shelfino_power_srv.request.data = false;
    shelfino_power.call(shelfino_power_srv);
    ROS_INFO("Shelfino engines OFF");
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_controller_node");
    ros::NodeHandle controller_node;

    // Initialize controller
    ShelfinoController controller(0.3, 0.3, 1000.0);
    ros::Duration(2.0).sleep(); // do not remove this sleep (necessary to reset odometry)
    controller.reset_odometry();
    controller_ptr = &controller;

    // Manage engines
    shelfino_power = controller_node.serviceClient<std_srvs::SetBool>("/shelfino2/power");
    signal(SIGINT, handler);
    
    // Engines Power on
    shelfino_power_srv.request.data = true;
    shelfino_power.call(shelfino_power_srv);
    if (shelfino_power_srv.response.success)
        ROS_INFO("Shelfino engines ON");
    else
        ROS_WARN("Cannot start Shelfino engines");

    // Advertise services and keep listening
    ros::ServiceServer move_service = controller_node.advertiseService("shelfino/move_to", move_to);
    ros::ServiceServer rotate_service = controller_node.advertiseService("shelfino/rotate", rotate);
    ros::ServiceServer point_service = controller_node.advertiseService("shelfino/point_to", point_to);
    ros::ServiceServer forward_service = controller_node.advertiseService("shelfino/move_forward", move_forward);
    
    ros::spin();

    return 0;

}