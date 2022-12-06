#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_test");
    ros::NodeHandle node;

    ros::Publisher shelfino_pub = node.advertise<geometry_msgs::Twist>("/shelfino/velocity/command", 1000);

    return 0;
}