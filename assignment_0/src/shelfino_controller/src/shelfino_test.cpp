#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "kinematics_lib/shelfino_kinematics.h"

using namespace std;

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_test");
    
    ShelfinoController controller(0.2, 0.2, 1000.);

    Coordinates dest;
    dest << 4, 4, 0;
    controller.shelfino_move_to(dest, 0);

    dest << 0, 0, 0;
    controller.shelfino_move_to(dest, 0);

    return 0;
}