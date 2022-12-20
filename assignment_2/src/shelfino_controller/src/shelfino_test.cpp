#include "ros/ros.h"
#include "shelfino_controller/MoveTo.h"
#include "shelfino_controller/Rotate.h"
#include "shelfino_controller/MoveForward.h"
#include <vector>

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fsm_controller");
    ros::NodeHandle fsm_node;
    ros::Rate loop_rate(100.);

    shelfino_controller::MoveTo shelfino_move_srv;
    shelfino_controller::Rotate shelfino_rotate_srv;
    shelfino_controller::MoveForward shelfino_forward_srv;

    ros::ServiceClient shelfino_move_client = fsm_node.serviceClient<shelfino_controller::MoveTo>("shelfino/move_to");
    ros::ServiceClient shelfino_rotate_client = fsm_node.serviceClient<shelfino_controller::Rotate>("shelfino/rotate");
    ros::ServiceClient shelfino_forward_client = fsm_node.serviceClient<shelfino_controller::MoveForward>("shelfino/move_forward");

    vector<vector<double>> areas;
    areas.push_back({2.5, 1.8, 3}); // 3,3
    areas.push_back({4.5, 1.8, 3}); // 5,3
    areas.push_back({4.5, 3.8, 3}); // 5,5
    
    for (int current_area = 0; current_area < 3; current_area++)
    {
        // Move shelfino to the center of the current area
        shelfino_move_srv.request.pos.x = areas[current_area][0];
        shelfino_move_srv.request.pos.y = areas[current_area][1];
        shelfino_move_client.call(shelfino_move_srv);
        // Wait a bit, just to avoid shit
        ros::Duration(2.0).sleep();
    }
}
