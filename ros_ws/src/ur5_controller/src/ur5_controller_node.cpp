#include "ur5_controller/ur5_services.h"

UR5Controller *controller_ptr = nullptr;

bool srv_move_to(ur5_controller::MoveTo::Request &req, ur5_controller::MoveTo::Response &res)
{
    Coordinates pos;
    RotationMatrix rot;
    pos << req.pos.x, req.pos.y, req.pos.z;
    rot = euler_to_rot(req.rot.roll, req.rot.pitch, req.rot.yaw);

    res.status = controller_ptr->move_to(pos, rot, 50);

    return true;
}


bool srv_set_gripper(ur5_controller::SetGripper::Request &req, ur5_controller::SetGripper::Response &res)
{
    controller_ptr->set_gripper(req.diameter);

    return true;
}


int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_controller_node");
    ros::NodeHandle controller_node;

    UR5Controller controller(1000.0, 0.05, 10.0);
    controller_ptr = &controller;

    ros::ServiceServer move_service = controller_node.advertiseService("ur5/move_to", srv_move_to);
    ros::ServiceServer gripper_service = controller_node.advertiseService("ur5/set_gripper", srv_set_gripper);
    ros::spin();

    return 0;
}
