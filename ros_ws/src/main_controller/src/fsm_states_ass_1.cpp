#include "main_controller/fsm.h"
#include <string> 

/* Extern variables */

extern State_t current_state;
extern std::vector<std::vector<double>> areas;
extern ros::ServiceClient detection_client;
extern ros::ServiceClient vision_stop_client;

extern shelfino_controller::Coordinates shelfino_current_pos;
extern double shelfino_current_rot;

/* Global variables */

namespace ass_1 {
    geometry_msgs::Pose block_load_pos;
    robotic_vision::Detect detection_srv;
    robotic_vision::Stop vision_stop_srv;

    int current_area_index; // Index of the current area in the areas array (different to area number)
    int current_block_class;
    double current_block_distance;
    double current_block_angle; // If the block is not centered in front of shelfino
}

void ass_1::init(void)
{
    // Global FSM variables
    current_area_index = 0;
    current_block_class = -1;
    current_block_distance = 0;
    
    shelfino_current_pos.x = 0;
    shelfino_current_pos.y = 0;
    shelfino_current_rot = 0;
    
    // This is for gazebo
    block_load_pos.position.x = 0.5;
    block_load_pos.position.y = 0.7;
    block_load_pos.position.z = 0.87;
    block_load_pos.orientation.w = 0.706;
    block_load_pos.orientation.z = 0.706;

    current_state = STATE_SHELFINO_ROTATE_AREA;
}

void ass_1::shelfino_rotate_towards_next_area(void)
{
    ROS_DEBUG("Rotating towards area %d", (int)areas[current_area_index][3]);
    shelfino_point_to(areas[current_area_index][0], areas[current_area_index][1]);

    // Service call to block detection node
    detection_client.call(detection_srv);
    if (detection_srv.response.status == 1)
    {
        ROS_INFO("Block identified!");
        current_block_distance = detection_srv.response.box.distance - 0.50;
        current_block_angle = (320.0 - (double)(detection_srv.response.box.xmax + detection_srv.response.box.xmin) / 2.0) / 320.0 * (M_PI / 4.0);
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
    // Make service call to python detection node
    // call returns distance to block
    // if distance > area radius, wrong block
    detection_client.call(detection_srv);
    // if response is valid:
    if (detection_srv.response.status == 1)
    {
        ROS_INFO("Block identified!");
        current_block_distance = detection_srv.response.box.distance - 0.50;
        current_block_class = detection_srv.response.box.class_n;
        current_block_angle = (320.0 - (double)(detection_srv.response.box.xmax + detection_srv.response.box.xmin) / 2.0) / 320.0 * (M_PI / 4.0);
        current_state = STATE_SHELFINO_CHECK_BLOCK;
        return;
    }
 
    // Rotate shelfino on its position
    shelfino_rotate(2 * M_PI);
}

void ass_1::shelfino_check_block(void)
{
    // // Rotate shelfino if block is not centered in front of him
    // shelfino_rotate(current_block_angle);

    // // Move shelfino forward to detected block
    // shelfino_forward(current_block_distance - 0.5, false);

    shelfino_move_to(
        shelfino_current_pos.x + current_block_distance * cos(current_block_angle + shelfino_current_rot),
        shelfino_current_pos.y + current_block_distance * sin(current_block_angle + shelfino_current_rot),
        0
    );

    ros::Duration(1.0).sleep();
    detection_client.call(detection_srv);
    if (detection_srv.response.status == 1)
    {
        current_block_class = detection_srv.response.box.class_n;
    }
    
    ROS_INFO("Block classified: %d", current_block_class);
    vision_stop_client.call(vision_stop_srv);

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