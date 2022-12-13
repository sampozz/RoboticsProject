#include "ros/ros.h"
#include "main_controller/fsm.h"


void init()
{
    gazebo_link_attacher.call(link_attacher_srv);
    // Initial and waiting position
    home_pos.x = 0.1;
    home_pos.y = -0.2;
    home_pos.z = 0.4;
    // Where to find a megablock from mobile robot
    load_pos.x = 0.0;
    load_pos.y = -0.35;
    load_pos.z = 0.73;
    // Basket test
    unload_pos.x = 0.45;
    unload_pos.y = 0.1;
    unload_pos.z = 0.6;

    current_state = STATE_UR5_HOME;
}

void ur5_homing()
{
    // Move ur5 to home position
    ur5_move_srv.request.pos = home_pos;
    ur5_move_srv.request.rot = default_rot;
    ur5_move_client.call(ur5_move_srv);

    current_state = STATE_UR5_LOAD;
}

void ur5_load()
{
    // Open gripper
    ur5_gripper_srv.request.diameter = 100;
    ur5_gripper_client.call(ur5_gripper_srv);
    // Move ur5 to load position
    ur5_move_srv.request.pos = load_pos;
    ur5_move_srv.request.rot = default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Grab
    ur5_gripper_srv.request.diameter = 31;
    ur5_gripper_client.call(ur5_gripper_srv);
    // attach();

    current_state = STATE_UR5_UNLOAD;
}

void ur5_unload()
{
    // Move ur5 to home position
    ur5_move_srv.request.pos = home_pos;
    ur5_move_srv.request.rot = default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Move ur5 to unload position
    ur5_move_srv.request.pos = unload_pos;
    ur5_move_srv.request.rot = default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Open gripper
    // detach();
    ur5_gripper_srv.request.diameter = 100;
    ur5_gripper_client.call(ur5_gripper_srv);

    current_state = STATE_SHELFINO_TEST;
}

void shelfino_test()
{
    shelfino_move_srv.request.pos.x = 3;
    shelfino_move_srv.request.pos.y = 3;
    shelfino_move_client.call(shelfino_move_srv);

    current_state = STATE_END;
}

void attach()
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = "lego";
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_attacher.call(link_attacher_srv);
}

void detach()
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = "lego";
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_detacher.call(link_attacher_srv);
}