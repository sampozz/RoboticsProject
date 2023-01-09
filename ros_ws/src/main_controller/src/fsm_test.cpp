#include "main_controller/fsm.h"
#include <string> 

/* Services */

extern ros::ServiceClient detection_client;
extern ros::ServiceClient vision_stop_client;

extern robotic_vision::Detect detection_srv;
extern robotic_vision::Ping vision_stop_srv;

/* Global state variables (defined into fsm_utils.cpp) */

extern State_t current_state;
extern std::vector<std::vector<double>> areas;

extern geometry_msgs::Pose block_load_pos;
extern ur5_controller::Coordinates ur5_home_pos, ur5_load_pos, ur5_unload_pos;
extern ur5_controller::EulerRotation ur5_default_rot;
extern shelfino_controller::Coordinates shelfino_current_pos, block_pos;
extern double shelfino_current_rot;
extern int current_area_index; 
extern robotic_vision::BoundingBox block_shelfino;
extern double block_angle; 

extern std::vector<double> unload_pos_y;
extern std::map<int, int> class_to_basket_map;

void state_test(void)
{    
    srand(time(0));
    // Initial and park position
    ur5_home_pos.x = 0.1;
    ur5_home_pos.y = -0.3;
    ur5_home_pos.z = 0.4;
    ur5_default_rot.roll = M_PI / 2;
    
    // Where to load the megablock
    ur5_load_pos.x = 0.0;
    ur5_load_pos.y = -0.35;
    ur5_load_pos.z = 0.73;
    
    // Where to find baskets
    ur5_unload_pos.x = 0.38;
    ur5_unload_pos.z = 0.55;
    unload_pos_y.push_back(0.12);
    unload_pos_y.push_back(-0.03);
    unload_pos_y.push_back(-0.18);
    unload_pos_y.push_back(-0.33);

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);

    //////////
    // LOAD

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    // Open gripper
    ur5_grip(100);
    // Move ur5 to load position
    ur5_move(ur5_load_pos, ur5_default_rot);
    // Grab
    ur5_grip(31);


    //////////
    // UNLOAD

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    // Move ur5 to unload position
    ur5_unload_pos.y = unload_pos_y[rand() % 4];
    ur5_move(ur5_unload_pos, ur5_default_rot);
    // Open gripper
    ur5_grip(100);
}