#include "main_controller/fsm.h"

shelfino_controller::MoveTo shelfino_move_srv;
shelfino_controller::Rotate shelfino_rotate_srv;
shelfino_controller::PointTo shelfino_point_srv;
shelfino_controller::MoveForward shelfino_forward_srv;
gazebo_ros_link_attacher::Attach link_attacher_srv;

ros::ServiceClient shelfino_move_client, shelfino_point_client,
    shelfino_rotate_client, shelfino_forward_client, 
    gazebo_link_attacher, gazebo_link_detacher;

extern shelfino_controller::Coordinates shelfino_current_pos;

void shelfino_forward(double distance, bool control)
{
    shelfino_forward_srv.request.distance = distance;
    shelfino_forward_srv.request.control = control;

    shelfino_forward_client.call(shelfino_forward_srv);

    shelfino_current_pos.x = shelfino_forward_srv.response.pos.x;
    shelfino_current_pos.y = shelfino_forward_srv.response.pos.y;
}

void shelfino_rotate(double angle)
{
    shelfino_rotate_srv.request.angle = angle;
    shelfino_rotate_client.call(shelfino_rotate_srv);
}

void shelfino_point_to(double x, double y)
{
    shelfino_point_srv.request.pos.x = x;
    shelfino_point_srv.request.pos.y = y;
    shelfino_point_client.call(shelfino_point_srv);
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