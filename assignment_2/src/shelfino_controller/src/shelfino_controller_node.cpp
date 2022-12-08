#include "ros/ros.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "shelfino_controller/MoveTo.h"

ShelfinoController *controller_ptr = nullptr;

bool move_to(shelfino_controller::MoveTo::Request &req, shelfino_controller::MoveTo::Response &res)
{
    Coordinates pos;
    pos << req.pos.x, req.pos.y, 0;
    
    controller_ptr->shelfino_move_to(pos, req.rot);
    return true;
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_controller_node");
    ros::NodeHandle controller_node;

    ShelfinoController controller(0.2, 0.2, 1000.0);
    controller_ptr = &controller;

    ros::ServiceServer move_service = controller_node.advertiseService("shelfino/move_to", move_to);
    
    ros::spin();

    return 0;

}