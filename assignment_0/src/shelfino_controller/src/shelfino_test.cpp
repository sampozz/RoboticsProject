#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "shelfino_controller/shelfino_controller_lib.h"
#include "kinematics_lib/shelfino_kinematics.h"

using namespace std;

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_test");
    
    ShelfinoController controller(0.1, 0.1, 1000.);

    Coordinates dest;
    dest << 5, 5, 0;

    controller.shelfino_move_to(dest, 0);

    return 0;
}