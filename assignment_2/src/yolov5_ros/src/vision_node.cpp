#include "ros/ros.h"
#include "yolov5_ros/Detect.h"
#include "yolov5_ros/BoundingBoxes.h"
#include "yolov5_ros/BoundingBox.h"

bool block_detected = false;
double detection_timestamp = 0;
yolov5_ros::BoundingBox block;

void yolo_callback(const yolov5_ros::BoundingBoxes::ConstPtr &msg)
{
    // TODO: if n > 1 return nearest
    if (msg->n > 0)
    {
        // Block detected
        block_detected = true;
        detection_timestamp = ros::Time::now().toSec();
        block = msg->bounding_boxes[0];
    }
}

bool detect(yolov5_ros::Detect::Request &req, yolov5_ros::Detect::Response &res)
{
    if (block_detected && ros::Time::now().toSec() - detection_timestamp < 5)
    {
        res.box = block;
        res.status = 1;
    }
    
    block_detected = false;
    return true;    
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle vision_node;

    ros::Subscriber yolo_detection_sub = vision_node.subscribe("/yolov5/detections", 10, yolo_callback);

    ros::ServiceServer detection_service = vision_node.advertiseService("vision/detect", detect);
    ros::spin();

    return 0;
}
