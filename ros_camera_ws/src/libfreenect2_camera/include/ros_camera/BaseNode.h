/*************************************************************************************//**
 *  @file       BaseNode.h
 *
 *  @brief      Brief description of BaseNode.h
 *
 *  @date       2016-08-02 16:53
 *         
 **************************************************************************************/


#ifndef BASENODE_H
#define BASENODE_H
#include <ros/ros.h>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>

#include <tf/transform_broadcaster.h>

class BaseNode{
public:
  BaseNode();
  virtual bool Init();

protected:
  virtual void prepareTopic();    
  virtual void prepareTrnasform();
  virtual void TopicPublisher();
  virtual void StreamdatatoMsg();
  virtual void SetCameraInfoMsg();
  ros::NodeHandle n;
  ros::Publisher depth_pub, color_pub;
  ros::Publisher depthCamInfo_pub, colorCamInfo_pub;
  std_msgs::Header h_color, h_depth;
  sensor_msgs::Image msg_ColorImage, msg_DepthImage;
  tf::TransformBroadcaster *tfbr;
};


#endif /* !BASENODE_H */

