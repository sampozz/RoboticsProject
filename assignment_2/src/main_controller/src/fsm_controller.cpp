#include "ur5_controller_lib/ur5_controller.h"
#include "main_controller/fsm.h"

using namespace std;

UR5Controller *ur5_controller_ptr = NULL;
Coordinates home_pos, load_pos, unload_pos;
RotationMatrix default_rot;

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
    
    // Init ur5 controller 
    UR5Controller ur5_controller(1000.0, 0.005, 10.0);
    ur5_controller_ptr = &ur5_controller;

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
    
    // Default rotation matrix, gripper points downward
    euler_to_rot(0, 0, 0, default_rot);
    // Initial and waiting position
    home_pos << 0.1, -0.2, 0.4;
    // Where to find a megablock from mobile robot
    load_pos << 0.0, -0.35, 0.7;
    unload_pos << 0.45, 0.1, 0.6;

    current_state = STATE_UR5_HOME;
}

void ur5_homing()
{
    // Move ur5 to home position
    ur5_controller_ptr->ur5_follow_path(home_pos, default_rot, 20);

    current_state = STATE_UR5_LOAD;
}

void ur5_load()
{
    // Move ur5 to load position
    ur5_controller_ptr->ur5_follow_path(load_pos, default_rot, 20);
    // Grab
    ur5_controller_ptr->ur5_set_gripper(20);

    current_state = STATE_UR5_UNLOAD;
}

void ur5_unload()
{
    // Move ur5 to home position
    ur5_controller_ptr->ur5_follow_path(home_pos, default_rot, 20);
    // Move ur5 to unload position
    ur5_controller_ptr->ur5_follow_path(unload_pos, default_rot, 20);
    // Grab
    ur5_controller_ptr->ur5_set_gripper(100);

    current_state = STATE_UR5_HOME;
}