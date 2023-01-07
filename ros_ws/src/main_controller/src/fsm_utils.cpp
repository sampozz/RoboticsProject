#include "main_controller/fsm.h"

/* Global Service Clients */

ros::ServiceClient shelfino_move_client, shelfino_point_client,
    shelfino_rotate_client, shelfino_forward_client, 
    gazebo_link_attacher, gazebo_link_detacher,
    ur5_move_client, ur5_gripper_client,
    vision_stop_client, pointcloud_client,
    detection_client, gazebo_set_state,
    gazebo_get_state;

/* Global Shelfino SRV Variables */

shelfino_controller::MoveTo shelfino_move_srv;
shelfino_controller::Rotate shelfino_rotate_srv;
shelfino_controller::PointTo shelfino_point_srv;
shelfino_controller::MoveForward shelfino_forward_srv;

/* Global UR5 SRV Variables */

ur5_controller::MoveTo ur5_move_srv;
ur5_controller::SetGripper ur5_gripper_srv;

/* Global vision SRV Variables */

robotic_vision::Detect detection_srv;
robotic_vision::Ping vision_stop_srv;
robotic_vision::PointCloud pointcloud_srv;

/* Global Gazebo SRV Variables */

gazebo_msgs::GetModelState get_state_srv;
gazebo_msgs::SetModelState set_state_srv;
gazebo_ros_link_attacher::Attach link_attacher_srv;

/* State global variables */

shelfino_controller::Coordinates shelfino_current_pos, block_pos;
ur5_controller::Coordinates ur5_home_pos, ur5_load_pos, ur5_unload_pos;
ur5_controller::EulerRotation ur5_default_rot;
geometry_msgs::Pose block_load_pos;
double shelfino_current_rot; // Shelfino current rotation angle
int current_area_index; // Index of the current area in the areas array (different to area number)
robotic_vision::BoundingBox block_shelfino; // Block detected and classified by shelfino
robotic_vision::BoundingBox block_ur5; // Block detected and classified by ur5
double block_angle; // If the block is not centered in front of shelfino
int choosen_block_class; // Class of the detected block choosen after classification from shelfino and ur5

std::vector<double> unload_pos_y;
std::map<int, int> class_to_basket_map;

void shelfino_move_to(double x, double y, double yaw)
{
    shelfino_move_srv.request.pos.x = x;
    shelfino_move_srv.request.pos.y = y;
    shelfino_move_srv.request.rot = yaw;

    shelfino_move_client.call(shelfino_move_srv);

    shelfino_current_pos.x = x;
    shelfino_current_pos.y = y;
    shelfino_current_rot = shelfino_move_srv.response.rot;
}

void shelfino_forward(double distance, bool control)
{
    shelfino_forward_srv.request.distance = distance - 0.15;
    shelfino_forward_srv.request.control = control;

    shelfino_forward_client.call(shelfino_forward_srv);

    shelfino_current_pos.x = shelfino_forward_srv.response.pos.x;
    shelfino_current_pos.y = shelfino_forward_srv.response.pos.y;
}

void shelfino_rotate(double angle)
{
    shelfino_rotate_srv.request.angle = angle;
    shelfino_rotate_client.call(shelfino_rotate_srv);
    shelfino_current_rot = shelfino_rotate_srv.response.rot;
}

void shelfino_point_to(double x, double y)
{
    shelfino_point_srv.request.pos.x = x;
    shelfino_point_srv.request.pos.y = y;
    shelfino_point_client.call(shelfino_point_srv);
    shelfino_current_rot = shelfino_point_srv.response.rot;
}

bool shelfino_detect(void)
{
    detection_client.call(detection_srv);
    if (detection_srv.response.status == 0)
        return false;

    block_shelfino = detection_srv.response.box;
    block_angle = (320.0 - (double)(detection_srv.response.box.xmax + detection_srv.response.box.xmin) / 2.0) / 320.0 * (M_PI / 6.0);
    block_pos.x = shelfino_current_pos.x + block_shelfino.distance * cos(block_angle + shelfino_current_rot) + 0.5;
    block_pos.y = shelfino_current_pos.y + block_shelfino.distance * sin(block_angle + shelfino_current_rot) + 1.2;
    block_shelfino.distance -= 0.50;
    return true;
}

bool ur5_move(ur5_controller::Coordinates& pos, ur5_controller::EulerRotation& rot)
{
    ur5_move_srv.request.pos = pos;
    ur5_move_srv.request.rot = rot;
    ur5_move_client.call(ur5_move_srv);

    return ur5_move_srv.response.status;
}

void ur5_grip(double diameter)
{
    ur5_gripper_srv.request.diameter = diameter;
    ur5_gripper_client.call(ur5_gripper_srv);
}

void attach(int model)
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = std::to_string(model);
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_attacher.call(link_attacher_srv);
}

void detach(int model)
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = std::to_string(model);
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_detacher.call(link_attacher_srv);
}