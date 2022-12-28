#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

using namespace std;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "shelfino_stop");
    ros::NodeHandle node;
    ros::Rate loop_rate(100.);

    ros::Publisher velocity_pub = node.advertise<geometry_msgs::Twist>("/shelfino/velocity/command", 1);

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    
    while(ros::ok())
    {
        velocity_pub.publish(msg);
    }
}
