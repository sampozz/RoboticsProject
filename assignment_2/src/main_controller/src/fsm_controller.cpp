#include "ros/ros.h"
#include "main_controller/fsm.h"
#include "ur5_controller/MoveTo.h"

using namespace std;

ur5_controller::Coordinates home_pos, load_pos, unload_pos;
ur5_controller::EulerRotation default_rot;

ur5_controller::MoveTo service;

ros::ServiceClient client;

void init();
void ur5_homing();
void ur5_load();
void ur5_unload();

StateMachine_t fsm[] = {
    {STATE_INIT, init},
    {STATE_UR5_HOME, ur5_homing},
    {STATE_UR5_LOAD, ur5_load},
    {STATE_UR5_UNLOAD, ur5_unload},
};

State_t current_state;

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "fsm_controller");
    ros::NodeHandle fsm_node;

    client = fsm_node.serviceClient<ur5_controller::MoveTo>("move_to");

    // Initial state
    current_state = STATE_INIT;

    // Wait a bit, just to avoid shit
    ros::Duration(0.5).sleep();

    while (ros::ok())
    {
        if (current_state < STATE_END)
        {
            cout << "Executing state function " << current_state << endl;
            (*fsm[current_state].state_function)();
        }
        else
        {
            break;
        }
    }

    return 0;
}

void init()
{
    // Initial and waiting position
    home_pos.x = 0.1;
    home_pos.y = -0.2;
    home_pos.z = 0.4;
    // Where to find a megablock from mobile robot
    load_pos.x = 0.0;
    load_pos.y = -0.35;
    load_pos.z = 0.7;
    // Basket test
    unload_pos.x = 0.45;
    unload_pos.y = 0.1;
    unload_pos.z = 0.6;

    current_state = STATE_UR5_HOME;
}

void ur5_homing()
{
    // Move ur5 to home position
    service.request.pos = home_pos;
    service.request.rot = default_rot;
    client.call(service);

    current_state = STATE_UR5_LOAD;
}

void ur5_load()
{
    // Move ur5 to load position
    service.request.pos = load_pos;
    service.request.rot = default_rot;
    client.call(service);
    // Grab

    current_state = STATE_UR5_UNLOAD;
}

void ur5_unload()
{
    // Move ur5 to home position
    service.request.pos = home_pos;
    service.request.rot = default_rot;
    client.call(service);
    // Move ur5 to unload position
    service.request.pos = unload_pos;
    service.request.rot = default_rot;
    client.call(service);
    // Grab

    current_state = STATE_UR5_HOME;
}