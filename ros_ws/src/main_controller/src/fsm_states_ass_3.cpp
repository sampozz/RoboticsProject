#include "main_controller/fsm.h"
#include <string> 

/* Services */

extern ros::ServiceClient detection_client;
extern ros::ServiceClient gazebo_set_state;	
extern ros::ServiceClient gazebo_get_state;	
extern ros::ServiceClient vision_stop_client;
extern ros::ServiceClient pointcloud_client;

extern robotic_vision::Detect detection_srv;
extern robotic_vision::Ping vision_stop_srv;
extern robotic_vision::PointCloud pointcloud_srv;
extern gazebo_msgs::GetModelState get_state_srv;
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
extern robotic_vision::BoundingBox block_ur5;
extern double block_angle; 
extern int choosen_block_class;

extern std::vector<double> unload_pos_y;
extern std::map<int, int> class_to_basket_map;

void ass_3::init(void)
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
    
    // Where to find baskets
    ur5_unload_pos.x = 0.42;
    ur5_unload_pos.z = 0.55;
    unload_pos_y.push_back(0.12);
    unload_pos_y.push_back(-0.03);
    unload_pos_y.push_back(-0.18);
    unload_pos_y.push_back(-0.33);

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);

    current_state = STATE_SHELFINO_ROTATE_AREA;
    // current_state = STATE_UR5_LOAD;
}

void ass_3::shelfino_rotate_towards_next_area(void)
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

void ass_3::shelfino_next_area(void)
{
    ROS_INFO("Proceeding to area %d", (int)areas[current_area_index][3]);
    
    // Move shelfino to the center of the current area
    double distance = sqrt(pow(shelfino_current_pos.x - areas[current_area_index][0], 2) + 
        pow(shelfino_current_pos.y - areas[current_area_index][1], 2));
        
    shelfino_forward(distance, true);

    current_state = STATE_SHELFINO_SEARCH_BLOCK;
}

void ass_3::shelfino_search_block(void)
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

void ass_3::shelfino_check_block(void)
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
        ROS_INFO("Could not find the area associated to the detected object.");	
        current_state = STATE_SHELFINO_ROTATE_AREA;	
        return;	
    }

    current_state = STATE_SHELFINO_PARK;
}

void ass_3::shelfino_park(void)
{
    // gazebo move block on top of shelfino
    get_state_srv.request.model_name = "shelfino";
    gazebo_get_state.call(get_state_srv);

    set_state_srv.request.model_state.pose = get_state_srv.response.pose;
    set_state_srv.request.model_state.model_name = std::to_string((int)areas[current_area_index][3]);
    set_state_srv.request.model_state.pose.position.x -= 0.1;
    set_state_srv.request.model_state.pose.position.y += 0.1;
    set_state_srv.request.model_state.pose.position.z = 0.9;
    set_state_srv.request.model_state.pose.orientation.w = cos((shelfino_current_rot + M_PI / 2) / 2);
    set_state_srv.request.model_state.pose.orientation.z = sin((shelfino_current_rot + M_PI / 2) / 2);
    gazebo_set_state.call(set_state_srv);
    ros::Duration(1.0).sleep();

    attach((int)areas[current_area_index][3], false);
    shelfino_move_to(-0.2, -0.1, M_PI + 0.1);
    detach((int)areas[current_area_index][3], false);
    
    current_state = STATE_UR5_LOAD;
}

void ass_3::ur5_load(void)
{
    // Move ur5 to load position
    pointcloud_client.call(pointcloud_srv);
    if (pointcloud_srv.response.box.class_n != -1)
    {
        ur5_load_pos.x = pointcloud_srv.response.wx - 0.5;
        ur5_load_pos.y = 0.35 - pointcloud_srv.response.wy;
        ur5_load_pos.z = 0.8;
        block_ur5 = pointcloud_srv.response.box;
        ROS_DEBUG("Response from pointcloud: %f %f %f", ur5_load_pos.x, ur5_load_pos.y, ur5_load_pos.z);
    }
    else
    {
        ROS_WARN("UR5 could not find object. Cannot proceed.");
        ros::Duration(1.0).sleep();
        // TODO? Shake shelfino to let ur5 find the block
        return;
    }

    ROS_INFO("UR5 classified the object: %s", block_ur5.Class.data());
    
    // If the two cameras classified differently
    choosen_block_class = block_shelfino.class_n;
    if (block_shelfino.class_n != block_ur5.class_n)
    {
        if (block_shelfino.probability >= block_ur5.probability)
        {
            ROS_INFO("Classification from Shelfino was more accurate, using class: %s", block_shelfino.Class.data());
            choosen_block_class = block_shelfino.class_n;
        }
        else
        {
            ROS_INFO("Classification from UR5 was more accurate, using class: %s", block_ur5.Class.data());
            choosen_block_class = block_ur5.class_n;
        }
    }

    // Choose the right basket based on the block class
    if (class_to_basket_map.find(choosen_block_class) == class_to_basket_map.end())
    {
        // Use an empy basket
        class_to_basket_map.insert(std::pair<int, int>(choosen_block_class, class_to_basket_map.size()));
        // Else, an object of the same class has already been classified: put it in the same basket
    }

    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    ur5_grip(100);

    // Move UR5 to load position
    if (!ur5_move(ur5_load_pos, ur5_default_rot))
    {
        // If UR5 cannot find a path, try an intermediate position
        ur5_controller::Coordinates intermediate_pos;
        intermediate_pos.x = -0.1;
        intermediate_pos.y = -0.4;
        intermediate_pos.z = 0.55;
        ur5_move(intermediate_pos, ur5_default_rot);

        if (!ur5_move(ur5_load_pos, ur5_default_rot))
        {
            ROS_WARN("UR5 cannot move to the specified area.");
            ros::Duration(1.0).sleep();
            return;
        } 
    }

    // Grab
    // TODO: close the gripper based on block class
    ur5_grip(31);
    attach((int)areas[current_area_index][3], true);

    current_state = STATE_UR5_UNLOAD;
}

void ass_3::ur5_unload(void)
{
    // Move ur5 to home position
    ur5_move(ur5_home_pos, ur5_default_rot);
    // Move ur5 to unload position
    ur5_unload_pos.y = unload_pos_y[class_to_basket_map[choosen_block_class]];
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