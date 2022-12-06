#include "ros/ros.h"
#include "ur5_controller/ur5_controller_lib.h"
#include "ur5_controller/MoveTo.h"

UR5Controller *controller_ptr = nullptr;

bool move_to(ur5_controller::MoveTo::Request &req, ur5_controller::MoveTo::Response &res)
{
    std::cout << "Received request\n";
    Coordinates pos;
    RotationMatrix rot;
    pos << req.pos.x, req.pos.y, req.pos.z;
    euler_to_rot(req.rot.roll, req.rot.pitch, req.rot.yaw, rot);

    res.status = controller_ptr->ur5_follow_path(pos, rot, 20);

    return true;
}


int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle controller_node;

    UR5Controller controller(1000.0, 0.005, 10.0);
    controller_ptr = &controller;

    ros::ServiceServer service = controller_node.advertiseService("move_to", move_to);
    ros::spin();

    return 0;
}
