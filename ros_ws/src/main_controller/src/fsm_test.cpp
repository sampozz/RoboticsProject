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

extern shelfino_controller::Coordinates shelfino_current_pos, block_pos;
extern double shelfino_current_rot;
extern int current_area_index; 
extern robotic_vision::BoundingBox block_shelfino;
extern double block_angle; 

void state_test(void)
{
    shelfino_move_to(0, 1, 0);
    shelfino_move_to(-2, 1, 0);
}