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

void ass_1::init(void)
{
    // Global FSM variables
    current_area_index = 0;
    
    shelfino_current_pos.x = 0;
    shelfino_current_pos.y = 0;
    shelfino_current_rot = 0;

    current_state = STATE_SHELFINO_ROTATE_AREA;
}

void ass_1::shelfino_rotate_towards_next_area(void)
{
    ROS_DEBUG("Rotating towards area %d", (int)areas[current_area_index][3]);
    shelfino_point_to(areas[current_area_index][0], areas[current_area_index][1]);

    // Service call to block detection node
    if (shelfino_detect())
    {
        ROS_INFO("Block identified!");
        current_state = STATE_SHELFINO_CHECK_BLOCK;   
    }
    else
    {
        current_state = STATE_SHELFINO_NEXT_AREA;
    } 
}

void ass_1::shelfino_next_area(void)
{
    ROS_INFO("Proceeding to area %d", (int)areas[current_area_index][3]);
    
    // Move shelfino to the center of the current area
    double distance = sqrt(pow(shelfino_current_pos.x - areas[current_area_index][0], 2) + 
        pow(shelfino_current_pos.y - areas[current_area_index][1], 2));
        
    shelfino_forward(distance, true);

    current_state = STATE_SHELFINO_SEARCH_BLOCK;
}

void ass_1::shelfino_search_block(void)
{
    // Service call to block detection node
    if (shelfino_detect())
    {
        ROS_INFO("Block identified!");
        current_state = STATE_SHELFINO_CHECK_BLOCK;
        return;
    }
 
    // Rotate shelfino on its position
    shelfino_rotate(2 * M_PI);
}

void ass_1::shelfino_check_block(void)
{
    shelfino_move_to(
        shelfino_current_pos.x + block_shelfino.distance * cos(block_angle + shelfino_current_rot),
        shelfino_current_pos.y + block_shelfino.distance * sin(block_angle + shelfino_current_rot),
        0
    );

    ros::Duration(1.0).sleep();
    detection_client.call(detection_srv);
    if (detection_srv.response.status == 1 && detection_srv.response.box.probability > block_shelfino.probability)
    {
        block_shelfino = detection_srv.response.box;
    }
    
    ROS_INFO("Block classified: %s, position: (%f, %f)", block_shelfino.Class.data(), block_pos.x, block_pos.y);
    vision_stop_client.call(vision_stop_srv); // Blacklist this block

    // Check in which area shelfino is
    // TODO: block position should be used instead of shelfino position
    bool area_found = false;
    for (int i = 0; i < areas.size(); i++)
    {
        double dist = sqrt(pow(shelfino_current_pos.x - areas[i][0], 2) + 
            pow(shelfino_current_pos.y - areas[i][1], 2));
        
        if (dist < areas[i][2] + 0.2)
        {
            current_area_index = i;
            area_found = true;
            break;
        }
    }

    if (!area_found){
        ROS_INFO("Could not find the area associated to the detected block.");
        current_state = STATE_SHELFINO_ROTATE_AREA;
        return;
    }

    ROS_INFO("Completed area %d, %ld remaining", (int)areas[current_area_index][3], areas.size() - 1);
    areas.erase(areas.begin() + current_area_index);
    current_area_index = 0;

    if (areas.size() == 0)
        current_state = STATE_END;
    else
        current_state = STATE_SHELFINO_ROTATE_AREA;
}