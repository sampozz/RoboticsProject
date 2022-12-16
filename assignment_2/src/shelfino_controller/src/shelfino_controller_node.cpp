#include "ros/ros.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/MoveForward.h"

ShelfinoController *controller_ptr = nullptr;

bool move_to(shelfino_controller::MoveTo::Request &req, shelfino_controller::MoveTo::Response &res)
{
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;
    
    controller_ptr->shelfino_move_to(pos, req.rot);
    return true;
}

bool rotate(shelfino_controller::Rotate::Request &req, shelfino_controller::Rotate::Response &res)
{    
    controller_ptr->shelfino_rotate(req.angle);
    return true;
}

bool move_forward(shelfino_controller::MoveForward::Request &req, shelfino_controller::MoveForward::Response &res)
{    
    controller_ptr->shelfino_move_forward(req.distance);
    return true;
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_controller_node");
    ros::NodeHandle controller_node;

    ShelfinoController controller(0.3, 0.3, 1000.0);
    controller_ptr = &controller;

    ros::ServiceServer move_service = controller_node.advertiseService("shelfino/move_to", move_to);
    ros::ServiceServer rotate_service = controller_node.advertiseService("shelfino/rotate", rotate);
    ros::ServiceServer forward_service = controller_node.advertiseService("shelfino/move_forward", move_forward);
    
    ros::spin();

    return 0;

}