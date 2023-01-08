#include "robotic_vision/shelfino_vision.h"

bool block_detected = false;
double detection_timestamp = 0;
robotic_vision::BoundingBox block;

double camera_angle = 1.07;

bool real_robot = false;

void yolo_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->n; i++)
    {
        // Block detected
        robotic_vision::BoundingBox b = msg->bounding_boxes[i];
        if (b.is_blacklisted || b.distance > 2.5)
        {
            ROS_DEBUG("Detected block is blacklisted or too far");
            continue;
        }

        if (i == 0)
            block = b;            

        // Return nearest block only
        if (b.distance < block.distance)
            block = b;
        
        block_detected = true;
        detection_timestamp = ros::Time::now().toSec();
    }
}

bool srv_shelfino_detect(robotic_vision::Detect::Request &req, robotic_vision::Detect::Response &res)
{
    if (block_detected && ros::Time::now().toSec() - detection_timestamp < 5)
    {
        // Adjust distance based on camera position
        if (real_robot)
            block.distance = block.distance * sin(camera_angle);
        else            
            block.distance = block.distance * sin(camera_angle) + 0.25;    

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

    ros::param::get("real_robot", real_robot);

    ros::Subscriber yolo_detection_sub = shelfino_yolo_node.subscribe("/shelfino/yolo/detections", 10, yolo_callback);

    ros::ServiceServer detection_service = shelfino_yolo_node.advertiseService("shelfino/yolo/detect", srv_shelfino_detect);

    ros::spin();

    return 0;
}
