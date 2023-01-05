/** 
* @file ur5_vision.h 
* @brief Header file for the UR5 vision node
*
* @author Samuele Pozzani
*
* @date 05/01/2023
*/

#ifndef __UR5_VISION__
#define __UR5_VISION__

#include "ros/ros.h"
#include "robotic_vision/PointCloud.h"
#include "robotic_vision/Ping.h"
#include "robotic_vision/BoundingBoxes.h"
#include "robotic_vision/BoundingBox.h"

/**
 * Handle callback from /ur5/yolo/detections ROS Topic.
 * If a block is detected, save the boundingbox into buffer with a timestamp. 
 * 
 * @param msg The message retrieved from topic
 */
void yolo_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg);

/**
 * Handle requests from ur5/yolo/detect ROS service.
 * Get last block detected on buffer, compute central point of the boundingbox, send request 
 * to /ur5/locosim/pointcloud to get block position with respect to the world frame.
 * 
 * @param req The service request, it is empty
 * @param res The service response, contains the coordinates of the detected block and its boundingbox object 
 */
bool srv_ur5_detect(robotic_vision::PointCloud::Request &req, robotic_vision::PointCloud::Response &res);

#endif