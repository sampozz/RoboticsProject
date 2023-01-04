#include "ros/ros.h"
#include "robotic_vision/PointCloud.h"
#include "robotic_vision/Ping.h"
#include "robotic_vision/BoundingBoxes.h"
#include "robotic_vision/BoundingBox.h"

bool block_detected = false;
double detection_timestamp = 0;
robotic_vision::BoundingBox block;

ros::ServiceClient pointcloud_client;

void yolo_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg)
{
    for (int i = 0; i < msg->n; i++)
    {
        // Is it a block or shelfino?
        robotic_vision::BoundingBox detected_block = msg->bounding_boxes[i];
        if ((detected_block.xmax - detected_block.xmin) * (detected_block.ymax - detected_block.ymin) > 5000)
        {
            ROS_DEBUG("Detected shelfino, probably");
            continue;
        }

        // Block detected
        ROS_DEBUG("Block detected from UR5");
        block_detected = true;
        block = msg->bounding_boxes[i];
        detection_timestamp = ros::Time::now().toSec();
        
    }
}

bool detect_srv(robotic_vision::PointCloud::Request &req, robotic_vision::PointCloud::Response &res)
{
    if (block_detected && ros::Time::now().toSec() - detection_timestamp < 3)
    {
        robotic_vision::PointCloud pointcloud_srv;
        pointcloud_srv.request.x = int((block.xmax + block.xmin) / 2);
        pointcloud_srv.request.y = int((block.ymax + block.ymin) / 2);

        pointcloud_client.call(pointcloud_srv);
        res = pointcloud_srv.response;
        res.state = 0;
    }
    else
    {
        res.state = 1;
        ROS_DEBUG("Could not identify block");
    }
    
    block_detected = false;
    return true;    
}

int main(int argc, char **argv)
{
    // ROS Node initialization
    ros::init(argc, argv, "ur5_yolo_node");
    ros::NodeHandle ur5_yolo_node;

    ros::Subscriber yolo_detection_sub = ur5_yolo_node.subscribe("/ur5/yolo/detections", 10, yolo_callback);
    ros::ServiceServer detection_service = ur5_yolo_node.advertiseService("ur5/yolo/detect", detect_srv);

    pointcloud_client = ur5_yolo_node.serviceClient<robotic_vision::PointCloud>("/ur5/locosim/pointcloud");

    ros::spin();

    return 0;
}
