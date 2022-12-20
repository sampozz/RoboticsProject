#include "ros/ros.h"
#include "main_controller/fsm.h"
#include <string> 


void init()
{
    // Global FSM variables
    current_area = 0;
    current_block_class = 0;
    current_block_distance = 0;
    
    // Initial and park position
    ur5_home_pos.x = 0.1;
    ur5_home_pos.y = -0.3;
    ur5_home_pos.z = 0.4;
    ur5_default_rot.roll = M_PI / 2;
    
    shelfino_home_pos.x = 0.5;
    shelfino_home_pos.y = 1.2;
    shelfino_move_srv.request.rot = 0;
    
    // Where to load the megablock
    ur5_load_pos.x = 0.0;
    ur5_load_pos.y = -0.35;
    ur5_load_pos.z = 0.73;
    
    // This is for gazebo
    block_load_pos.position.x = 0.5;
    block_load_pos.position.y = 0.7;
    block_load_pos.position.z = 0.87;
    
    // Where to find baskets
    ur5_unload_pos.x = 0.42;
    ur5_unload_pos.z = 0.55;
    unload_position_y.push_back(0.12);
    unload_position_y.push_back(-0.03);
    unload_position_y.push_back(-0.18);
    unload_position_y.push_back(-0.33);

    // Move ur5 to home position
    ur5_move_srv.request.pos = ur5_home_pos;
    ur5_move_srv.request.rot = ur5_default_rot;
    ur5_move_client.call(ur5_move_srv);

    current_state = STATE_SHELFINO_NEXT_AREA;
}

void shelfino_next_area()
{
    ROS_INFO("Proceeding to area %d", current_area);
    
    // Move shelfino to the center of the current area
    shelfino_move_srv.request.pos.x = areas[current_area][0];
    shelfino_move_srv.request.pos.y = areas[current_area][1];
    shelfino_move_client.call(shelfino_move_srv);

    current_state = STATE_SHELFINO_SEARCH_BLOCK;
}

int a = 0, b = rand() % 10;
void shelfino_search_block()
{
    // Rotate shelfino on its position
    shelfino_rotate_srv.request.angle = M_PI / 10;
    shelfino_rotate_client.call(shelfino_rotate_srv);

    // Make service call to python detection node
    // call returns distance to block
    // if distance > area radius, wrong block
    // if response is valid:
    if (++a == b) {
        ROS_INFO("Block identified!");
        b = rand() % 10;
        a = 0;
        current_block_distance = 1;
        current_state = STATE_SHELFINO_CHECK_BLOCK;
    }
}

void shelfino_check_block()
{
    // Move shelfino to detected block
    shelfino_forward_srv.request.distance = current_block_distance;
    shelfino_forward_client.call(shelfino_forward_srv); 

    // Make service call to python classification node
    // if response == true:
    current_block_class = rand() % 11;
    ROS_INFO("Block classified: %d", current_block_class);

    // Choose the right basket based on the block class
    if (class_to_basket_map.find(current_block_class) == class_to_basket_map.end())
    {
        // Use an empy basket
        class_to_basket_map.insert(std::pair<int, int>(current_block_class, class_to_basket_map.size()));
        // Else, an object of the same class has already been classified: put it in the same basket
    }
    
    // gazebo move block to ur5 load position
    model_state_srv.request.model_state.model_name = std::to_string(current_area);
    model_state_srv.request.model_state.pose = block_load_pos;
    gazebo_model_state.call(model_state_srv);

    current_area++;
    current_state = STATE_SHELFINO_NEXT_AREA;
}

void ur5_load()
{
    // Move ur5 to home position
    ur5_move_srv.request.pos = ur5_home_pos;
    ur5_move_srv.request.rot = ur5_default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Open gripper
    ur5_gripper_srv.request.diameter = 100;
    ur5_gripper_client.call(ur5_gripper_srv);
    // Move ur5 to load position
    ur5_move_srv.request.pos = ur5_load_pos;
    ur5_move_srv.request.rot = ur5_default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Grab
    ur5_gripper_srv.request.diameter = 31;
    ur5_gripper_client.call(ur5_gripper_srv);
    attach();

    current_state = STATE_UR5_UNLOAD;
}

void ur5_unload()
{
    // Move ur5 to home position
    ur5_move_srv.request.pos = ur5_home_pos;
    ur5_move_srv.request.rot = ur5_default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Move ur5 to unload position
    ur5_move_srv.request.pos = ur5_unload_pos;
    ur5_move_srv.request.pos.y = unload_position_y[class_to_basket_map[current_block_class]];
    ur5_move_srv.request.rot = ur5_default_rot;
    ur5_move_client.call(ur5_move_srv);
    // Open gripper
    detach();
    ur5_gripper_srv.request.diameter = 100;
    ur5_gripper_client.call(ur5_gripper_srv);

    if (++current_area == 4)
        current_state = STATE_END;
    else
        current_state = STATE_SHELFINO_NEXT_AREA;
}

void attach()
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = std::to_string(current_area);
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_attacher.call(link_attacher_srv);
}

void detach()
{
    link_attacher_srv.request.model_name_1 = "ur5";
    link_attacher_srv.request.link_name_1 = "hand_1_link";
    link_attacher_srv.request.model_name_2 = std::to_string(current_area);
    link_attacher_srv.request.link_name_2 = "link";
    gazebo_link_detacher.call(link_attacher_srv);
}