#include "ros/ros.h"
#include "robotic_vision/Detect.h"
#include "robotic_vision/Stop.h"
#include "robotic_vision/BoundingBoxes.h"
#include "robotic_vision/BoundingBox.h"

bool block_detected = false;
double detection_timestamp = 0;
robotic_vision::BoundingBox block;

double camera_angle = 1.07;

void yolo_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg)
{
    // TODO: if n > 1 return nearest
    if (msg->n > 0)
    {
        // Block detected
        block = msg->bounding_boxes[0];
        detection_timestamp = ros::Time::now().toSec();
        
        if (!block.is_blacklisted)
            block_detected = true;
        else
            ROS_DEBUG("Detected block is blacklisted");
    }
}

bool detect_srv(robotic_vision::Detect::Request &req, robotic_vision::Detect::Response &res)
{
    if (block_detected && ros::Time::now().toSec() - detection_timestamp < 5)
    {
        // Adjust distance based on camera position
        block.distance = block.distance * sin(camera_angle);

        res.box = block;
        res.status = 1;
    }
    
    block_detected = false;
    return true;    
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "shelfino_yolo_node");
    ros::NodeHandle shelfino_yolo_node;

    ros::Subscriber yolo_detection_sub = shelfino_yolo_node.subscribe("/shelfino/yolo/detections", 10, yolo_callback);

    ros::ServiceServer detection_service = shelfino_yolo_node.advertiseService("shelfino/yolo/detect", detect_srv);
    ros::spin();

    return 0;
}
