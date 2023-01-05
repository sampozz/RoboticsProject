/** 
* @file shelfino_vision.h 
* @brief Header file for the shelfino vision node
*
* @author Samuele Pozzani
*
* @date 05/01/2023
*/

#ifndef __SHELFINO_VISION__
#define __SHELFINO_VISION__

#include "ros/ros.h"
#include "robotic_vision/Detect.h"
#include "robotic_vision/Ping.h"
#include "robotic_vision/BoundingBoxes.h"
#include "robotic_vision/BoundingBox.h"

/**
 * Handle callback from /shelfino/yolo/detections ROS Topic.
 * If a block is detected, save the boundingbox into buffer with a timestamp. 
 * Also, check that the block is not blacklisted nor to far.
 * 
 * @param msg The message retrieved from topic
 */
void yolo_callback(const robotic_vision::BoundingBoxes::ConstPtr &msg);

/**
 * Handle requests from shelfino/yolo/detect ROS service.
 * If a block is buffered, compute distance and return its boundingbox
 * 
 * @param req The service request, it is empty
 * @param res The service response, contains the boundingbox of the detected block
 */
bool srv_shelfino_detect(robotic_vision::Detect::Request &req, robotic_vision::Detect::Response &res);

#endif