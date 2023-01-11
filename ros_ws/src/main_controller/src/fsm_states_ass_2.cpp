#include "main_controller/fsm.h"
#include <string> 

/* Services */

extern ros::ServiceClient detection_client;
extern ros::ServiceClient gazebo_set_state;	
extern ros::ServiceClient vision_stop_client;

extern robotic_vision::Detect detection_srv;
extern robotic_vision::Ping vision_stop_srv;
extern gazebo_msgs::SetModelState set_state_srv;

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

void ass_2::init(void)
{
    // Global FSM variables
    current_area_index = 0;
    
    // Initial and park position
    ur5_home_pos.x = 0.1;
    ur5_home_pos.y = -0.3;
    ur5_home_pos.z = 0.4;
    ur5_default_rot.roll = M_PI / 2;
    
    shelfino_current_pos.x = 0;
    shelfino_current_pos.y = 0;
    shelfino_current_rot = 0;
    
    // Where to load the megablock
    ur5_load_pos.x = 0.0;
    ur5_load_pos.y = -0.35;
    ur5_load_pos.z = 0.73;
    
    // This is for gazebo
    block_load_pos.position.x = 0.5;
    block_load_pos.position.y = 0.7;
    block_load_pos.position.z = 0.87;
    block_load_pos.orientation.w = 0.706;
    block_load_pos.orientation.z = 0.706;
    
    // Where to find baskets
    ur5_unload_pos.x = 0.38;
    ur5_unload_pos.z = 0.55;
    unload_pos_y.push_back(0.12);
    unload_pos_y.push_back(-0.03);
    unload_pos_y.push_back(-0.18);
    unload_pos_y.push_back(-0.33);

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);

    current_state = STATE_SHELFINO_ROTATE_AREA;
}

void ass_2::shelfino_rotate_towards_next_area(void)
{
    ROS_DEBUG("Rotating towards area %d", (int)areas[current_area_index][3]);
    shelfino_point_to(areas[current_area_index][0], areas[current_area_index][1]);

    // Service call to block detection node
    if (shelfino_detect())
    {
        ROS_INFO("Object identified");
        current_state = STATE_SHELFINO_CHECK_BLOCK;   
    }
    else
    {
        current_state = STATE_SHELFINO_NEXT_AREA;
    } 
}

void ass_2::shelfino_next_area(void)
{
    ROS_INFO("Proceeding to area %d", (int)areas[current_area_index][3]);
    
    // Move shelfino to the center of the current area
    double distance = sqrt(pow(shelfino_current_pos.x - areas[current_area_index][0], 2) + 
        pow(shelfino_current_pos.y - areas[current_area_index][1], 2));
        
    shelfino_forward(distance, true);

    current_state = STATE_SHELFINO_SEARCH_BLOCK;
}

void ass_2::shelfino_search_block(void)
{
    // Service call to block detection node
    if (shelfino_detect())
    {
        ROS_INFO("Object identified");
        current_state = STATE_SHELFINO_CHECK_BLOCK;
        return;
    }
	
    // Rotate shelfino on its position	
    shelfino_rotate(2 * M_PI);
}

void ass_2::shelfino_check_block(void)
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
    
    ROS_INFO("Object classified: %s, position: (%.2f, %.2f)", block_shelfino.Class.data(), block_pos.x, block_pos.y);
    vision_stop_client.call(vision_stop_srv);

    // Choose the right basket based on the block class
    if (class_to_basket_map.find(block_shelfino.class_n) == class_to_basket_map.end())
    {
        // Use an empy basket
        class_to_basket_map.insert(std::pair<int, int>(block_shelfino.class_n, class_to_basket_map.size()));
        // Else, an object of the same class has already been classified: put it in the same basket
    }

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
        ROS_INFO("Could not find the area associated to the detected object.");	
        current_state = STATE_SHELFINO_ROTATE_AREA;	
        return;	
    }
    
    // gazebo move block to ur5 load position
    set_state_srv.request.model_state.model_name = std::to_string((int)areas[current_area_index][3]);
    set_state_srv.request.model_state.pose = block_load_pos;
    gazebo_set_state.call(set_state_srv);

    current_state = STATE_UR5_LOAD;
}

void ass_2::ur5_load(void)
{
    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    // Open gripper
    ur5_grip(100);
    // Move ur5 to load position
    ur5_move(ur5_load_pos, ur5_default_rot);
    // Grab
    // TODO: close the gripper based on block class
    ur5_grip(31);
    attach((int)areas[current_area_index][3], true);

    current_state = STATE_UR5_UNLOAD;
}

void ass_2::ur5_unload(void)
{
    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    // Move ur5 to unload position
    ur5_unload_pos.y = unload_pos_y[class_to_basket_map[block_shelfino.class_n]];
    ur5_move(ur5_unload_pos, ur5_default_rot);

    // Open gripper
    detach((int)areas[current_area_index][3], true);
    ur5_grip(100);

    ROS_INFO("Completed area %d, %ld remaining", (int)areas[current_area_index][3], areas.size() - 1);
    areas.erase(areas.begin() + current_area_index);
    current_area_index = 0;

    if (areas.size() == 0)
        current_state = STATE_END;
    else
        current_state = STATE_SHELFINO_ROTATE_AREA;
}