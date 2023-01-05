#include "shelfino_controller/shelfino_services.h"

ShelfinoController *controller_ptr = nullptr;

std_srvs::SetBool shelfino_power_srv;
ros::ServiceClient shelfino_power;

bool move_to(shelfino_controller::MoveTo::Request &req, shelfino_controller::MoveTo::Response &res)
{
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;
    
    double angle = controller_ptr->move_to(pos, req.rot);
    res.rot = angle;
    return true;
}

bool rotate(shelfino_controller::Rotate::Request &req, shelfino_controller::Rotate::Response &res)
{    
    double angle = controller_ptr->rotate(req.angle);
    res.rot = angle;
    return true;
}

bool point_to(shelfino_controller::PointTo::Request &req, shelfino_controller::PointTo::Response &res)
{    
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;

    double angle = controller_ptr->point_to(pos);
    res.rot = angle;
    return true;
}

bool move_forward(shelfino_controller::MoveForward::Request &req, shelfino_controller::MoveForward::Response &res)
{    
    Coordinates new_position = controller_ptr->move_forward(req.distance, req.control);
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
    ShelfinoController controller(0.2, 0.2, 1000.0);
    ros::Duration(2.0).sleep(); // do not remove this sleep (necessary to reset odometry)
    // Reset odometry: shelfino thinks it is in position (0,0,0) after this node starts running
    // otherwise lyapunov control (and the movement functions generally) will have problems
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