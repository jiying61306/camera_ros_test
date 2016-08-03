/************************************************************************************//**
 *  @file       test.cpp
 *
 *  @brief      Brief descriptinon of test.cpp 
 *
 *  @date       2016-08-02 14:38
 *
 ***************************************************************************************/

#include <ros/ros.h>
#include <iostream>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

using namespace cv;
/* ======= Function ==================================================
 *   Name: main
 *   Description: main entry Function
 * =================================================================== 
 */
Mat MatColor, MatDepth;

ros::Publisher color_pub, depth_pub, cam_info_color_pub, cam_info_depth_pub;
std_msgs::Header h_color, h_depth;
sensor_msgs::Image msg_ColorImage, msg_DepthImage;
sensor_msgs::CameraInfo cam_info_color, cam_info_depth;

cv_bridge::CvImage out_msgc, out_msgd;

libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev = 0;
std::string serial = ""; 
libfreenect2::SyncMultiFrameListener *listenerPtr;
libfreenect2::FrameMap frames;

tf::TransformBroadcaster *tfbr;

void prepareTransforms(){
  tf::Vector3 color_t_vec(0, 0, 0);
  tf::Matrix3x3 color_r_mat(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
    );
  tf::Transform transform(color_r_mat, color_t_vec);
  tfbr->sendTransform(tf::StampedTransform(transform, h_color.stamp, "kinect2", "kinect2_color"));
 
  tf::Vector3 depth_t_vec(0, 0, 0);
  tf::Matrix3x3 depth_r_mat(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
    );
  tf::Transform dtransform(depth_r_mat, depth_t_vec);
  tfbr->sendTransform(tf::StampedTransform(dtransform, h_depth.stamp, "kinect2", "kinect2_depth"));


}
void setCamInfo(){
  //color
  cam_info_color.header = msg_ColorImage.header;
  cam_info_color.height = msg_ColorImage.height;
  cam_info_color.width = msg_ColorImage.width;

  cam_info_color.D.push_back(0);
  cam_info_color.D.push_back(0);
  cam_info_color.D.push_back(0);
  cam_info_color.D.push_back(0);
  cam_info_color.D.push_back(0);
  // cam_info_color.D = std::vector<double>(5, 0.0);
  auto colorIntrinsics = dev->getColorCameraParams();

  cam_info_color.K[0] = colorIntrinsics.fx;
  cam_info_color.K[1] = 0;
  cam_info_color.K[2] = colorIntrinsics.cx;
  cam_info_color.K[3] = 0;
  cam_info_color.K[4] = colorIntrinsics.fy;
  cam_info_color.K[5] = colorIntrinsics.cy;
  cam_info_color.K[6] = 0;
  cam_info_color.K[7] = 0;
  cam_info_color.K[8] = 1;

  cam_info_color.R[0] = 1.0;
  cam_info_color.R[1] = 0.0;
  cam_info_color.R[2] = 0.0;
  cam_info_color.R[3] = 0.0;
  cam_info_color.R[4] = 1.0;
  cam_info_color.R[5] = 0.0;
  cam_info_color.R[6] = 0.0;
  cam_info_color.R[7] = 0.0;
  cam_info_color.R[8] = 1.0;

  cam_info_color.P[0] = colorIntrinsics.fx;
  cam_info_color.P[1] = 0;
  cam_info_color.P[2] = colorIntrinsics.cx;
  cam_info_color.P[3] = 0;
  cam_info_color.P[4] = 0;
  cam_info_color.P[5] = colorIntrinsics.fy;
  cam_info_color.P[6] = colorIntrinsics.cy;
  cam_info_color.P[7] = 0;
  cam_info_color.P[8] = 0;
  cam_info_color.P[9] = 0;
  cam_info_color.P[10] = 1;
  cam_info_color.P[11] = 0;
  ////
  //depth

  cam_info_depth.header = msg_DepthImage.header;
  cam_info_depth.height = msg_DepthImage.height;
  cam_info_depth.width  = msg_DepthImage.width;
  
  auto depthIntrinsics = dev->getIrCameraParams();
  cam_info_depth.D.push_back(depthIntrinsics.k1);
  cam_info_depth.D.push_back(depthIntrinsics.k2);
  cam_info_depth.D.push_back(depthIntrinsics.p1);
  cam_info_depth.D.push_back(depthIntrinsics.p2);
  cam_info_depth.D.push_back(depthIntrinsics.k3);
  // cam_info_depth.D = std::vector<double>(5, 0.0);

  cam_info_depth.K[0] = depthIntrinsics.fx*2;
  cam_info_depth.K[1] = 0;
  cam_info_depth.K[2] = depthIntrinsics.cx*2;
  cam_info_depth.K[3] = 0;
  cam_info_depth.K[4] = depthIntrinsics.fy*2;
  cam_info_depth.K[5] = depthIntrinsics.cy*2;
  cam_info_depth.K[6] = 0;
  cam_info_depth.K[7] = 0;
  cam_info_depth.K[8] = 1;

  cam_info_depth.R[0] = 1.0;
  cam_info_depth.R[1] = 0.0;
  cam_info_depth.R[2] = 0.0;
  cam_info_depth.R[3] = 0.0;
  cam_info_depth.R[4] = 1.0;
  cam_info_depth.R[5] = 0.0;
  cam_info_depth.R[6] = 0.0;
  cam_info_depth.R[7] = 0.0;
  cam_info_depth.R[8] = 1.0;

  cam_info_depth.P[0] = depthIntrinsics.fx*2;
  cam_info_depth.P[1] = 0;
  cam_info_depth.P[2] = depthIntrinsics.cx*2;
  cam_info_depth.P[3] = 0;
  cam_info_depth.P[4] = 0;
  cam_info_depth.P[5] = depthIntrinsics.fy*2;
  cam_info_depth.P[6] = depthIntrinsics.cy*2;
  cam_info_depth.P[7] = 0;
  cam_info_depth.P[8] = 0;
  cam_info_depth.P[9] = 0;
  cam_info_depth.P[10] = 1;
  cam_info_depth.P[11] = 0;
}
void TopicPublisher(){
  color_pub.publish(msg_ColorImage);
  depth_pub.publish(msg_DepthImage);
  cam_info_color_pub.publish(cam_info_color);
  cam_info_depth_pub.publish(cam_info_depth);
}
void MattoMsg(){
  h_color.stamp = ros::Time::now(); 
  h_depth.stamp = ros::Time::now(); 
  out_msgc.header   = h_color; // Same timestamp and tf frame as input image
  out_msgc.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
  out_msgc.image    = MatColor; // Your cv::Mat
  msg_ColorImage = *(out_msgc.toImageMsg());
  
  out_msgd.header   = h_depth; // Same timestamp and tf frame as input image
  out_msgd.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
  out_msgd.image    = MatDepth; // Your cv::Mat
  msg_DepthImage = *(out_msgd.toImageMsg());

}
void getCvMat(libfreenect2::Frame *rgb, libfreenect2::Frame *depth ){
//  cv::Mat temp;
  cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(MatDepth);
  cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(MatColor);
//  temp.convertTo(MatColor, CV_8UC3); 
  MatDepth /= 2000.0f;
}
void run(){
  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame *undistorted =new libfreenect2::Frame(512, 424, 4);
  libfreenect2::Frame *registered =new libfreenect2::Frame(512, 424, 4);
 
  namedWindow("Color", CV_WINDOW_AUTOSIZE);
  namedWindow("Depth", CV_WINDOW_AUTOSIZE);
  while(ros::ok()){
    if(waitKey(1) == 27) break;
    if (!listenerPtr->waitForNewFrame(frames, 10*1000)){ // 10 sconds
      std::cout << "timeout!" << std::endl;
      return ;
    }
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
    
    registration->apply(rgb, depth, undistorted, registered);
    getCvMat(rgb , undistorted);
    imshow("Depth", MatDepth);
    imshow("Color", MatColor);
    MattoMsg();
    setCamInfo();
    prepareTransforms();//need to run in anothor thread;
    TopicPublisher();
    listenerPtr->release(frames);
  }
}
int main(int argc, char **argv){
  ros::init(argc, argv, "capture_image");
 
  ros::NodeHandle n;
  color_pub = n.advertise<sensor_msgs::Image>("/rs/color/image", 5);  
  depth_pub = n.advertise<sensor_msgs::Image>("/rs/depth/image", 5);
  cam_info_color_pub = n.advertise<sensor_msgs::CameraInfo>("/rs/color/cam_info", 5);  
  cam_info_depth_pub = n.advertise<sensor_msgs::CameraInfo>("/rs/depth/cam_info", 5);
 
  tfbr = new tf::TransformBroadcaster();

  if(freenect2.enumerateDevices() == 0){
    std::cout << "no device connected!" << std::endl;
    return -1;
  }
  if (serial == ""){
    serial = freenect2.getDefaultDeviceSerialNumber();
  }
  
  dev = freenect2.openDevice(serial);
  
  bool enable_rgb = true;
  bool enable_depth = true;


  int types = 0;
  if (enable_rgb)
    types |= libfreenect2::Frame::Color;
  if (enable_depth)
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
  libfreenect2::SyncMultiFrameListener listener(types);

  listenerPtr = &listener;
  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);

  if (enable_rgb && enable_depth){
    if (!dev->start())
      return -1;
  }
  else{
    if (!dev->startStreams(enable_rgb, enable_depth))
      return -1;
  }
  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
  libfreenect2::Frame *undistorted =new libfreenect2::Frame(512, 424, 4);
  libfreenect2::Frame *registered =new libfreenect2::Frame(512, 424, 4);
  
  run();//main loop

  dev->stop();
  dev->close();

  return 0;
}


