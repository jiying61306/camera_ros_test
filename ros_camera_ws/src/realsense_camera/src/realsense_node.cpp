/************************************************************************************//**
 *  @file       test.cpp
 *
 *  @brief      Brief descriptinon of test.cpp 
 *
 *  @date       2016-08-04 19:32
 *
 ***************************************************************************************/
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <librealsense/rs.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>

#include <tf/transform_broadcaster.h>

using namespace cv;
Mat MatColor, MatDepth;

ros::Publisher color_pub, depth_pub, cam_info_color_pub, cam_info_depth_pub;
std_msgs::Header h_color, h_depth;
sensor_msgs::Image msg_ColorImage, msg_DepthImage;
sensor_msgs::CameraInfo cam_info_color, cam_info_depth;

cv_bridge::CvImage out_msgc, out_msgd;

tf::TransformBroadcaster *tfbr;

rs_error * e = 0;
rs_device * dev;
rs_context * ctx ;
rs_device *rs_device_ = NULL;

const int RS_WIDTH = 640; //cvMat->imageMsg -> caminfoMsg
const int RS_HEIGHT = 480;

void check_error(){
    if(e)
    {
        printf("rs_error was raised when calling %s(%s):\n", rs_get_failed_function(e), rs_get_failed_args(e));
        printf("    %s\n", rs_get_error_message(e));
        exit(EXIT_FAILURE);
    }
}


void prepareTransforms(){
  tf::Vector3 color_t_vec(0, 0, 0);
  tf::Matrix3x3 color_r_mat(
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
    );
  tf::Transform transform(color_r_mat, color_t_vec);
  tfbr->sendTransform(tf::StampedTransform(transform, h_color.stamp, "kinect2", "kinect2_color"));

  rs_extrinsics z_extrinsic;
   // extrinsics are offsets between the cameras
  rs_get_device_extrinsics(rs_device_, RS_STREAM_DEPTH, RS_STREAM_COLOR, &z_extrinsic, &e);
  check_error();
  transform.setOrigin(tf::Vector3(z_extrinsic.translation[2], -z_extrinsic.translation[0], -z_extrinsic.translation[1]));
  transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  tfbr->sendTransform(tf::StampedTransform(transform, h_depth.stamp, "kinect2", "kinect2_depth"));
}
void setCamInfoMsg(){
  rs_intrinsics colorIntrinsics, depthIntrinsics;
  rs_get_stream_intrinsics(rs_device_, RS_STREAM_COLOR, &colorIntrinsics, &e);
  check_error();
  rs_get_stream_intrinsics(rs_device_, RS_STREAM_DEPTH, &depthIntrinsics, &e);
  check_error();

  //color
  cam_info_color.header = msg_ColorImage.header;
  cam_info_color.height = msg_ColorImage.height;
  cam_info_color.width = msg_ColorImage.width;

  for (int i = 0; i < 5; i++){
    cam_info_color.D.push_back(colorIntrinsics.coeffs[i]);
  }
  cam_info_color.K[0] = colorIntrinsics.fx;
  cam_info_color.K[1] = 0;
  cam_info_color.K[2] = colorIntrinsics.ppx;
  cam_info_color.K[3] = 0;
  cam_info_color.K[4] = colorIntrinsics.fy;
  cam_info_color.K[5] = colorIntrinsics.ppy;
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
  cam_info_color.P[2] = colorIntrinsics.ppx;
  cam_info_color.P[3] = 0;
  cam_info_color.P[4] = 0;
  cam_info_color.P[5] = colorIntrinsics.fy;
  cam_info_color.P[6] = colorIntrinsics.ppy;
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
 
  for (int i = 0; i < 5; i++){
    cam_info_depth.D.push_back(depthIntrinsics.coeffs[i]);
  }
  cam_info_depth.K[0] = depthIntrinsics.fx;
  cam_info_depth.K[1] = 0;
  cam_info_depth.K[2] = depthIntrinsics.ppx;
  cam_info_depth.K[3] = 0;
  cam_info_depth.K[4] = depthIntrinsics.fy;
  cam_info_depth.K[5] = depthIntrinsics.ppy;
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

  cam_info_depth.P[0] = depthIntrinsics.fx;
  cam_info_depth.P[1] = 0;
  cam_info_depth.P[2] = depthIntrinsics.ppx;
  cam_info_depth.P[3] = 0;
  cam_info_depth.P[4] = 0;
  cam_info_depth.P[5] = depthIntrinsics.fy;
  cam_info_depth.P[6] = depthIntrinsics.ppy;
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
void setImageMsg(){
  h_color.frame_id = "kinect2";
  h_depth.frame_id = "kinect2";
  msg_ColorImage.header = h_color;
  msg_ColorImage.height = RS_HEIGHT;
  msg_ColorImage.width  = RS_WIDTH;
  h_color.stamp = ros::Time::now(); 
  h_depth.stamp = ros::Time::now(); 
  out_msgc.header   = h_color; // Same timestamp and tf frame as input image
  out_msgc.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
  out_msgc.image    = MatColor; // Your cv::Mat
  msg_ColorImage = *(out_msgc.toImageMsg());
  
  msg_DepthImage.header = h_depth;
  msg_DepthImage.height = RS_HEIGHT;
  msg_DepthImage.width  = RS_WIDTH;
  out_msgd.header   = h_depth; // Same timestamp and tf frame as input image
  out_msgd.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
  out_msgd.image    = MatDepth; // Your cv::Mat
  msg_DepthImage = *(out_msgd.toImageMsg());
}
void getStreamdata(float* dep_fra){
    rs_wait_for_frames(dev, &e);
    //color stream
    const uint16_t * depth_frame = (const uint16_t *)(rs_get_frame_data(dev, RS_STREAM_DEPTH, &e));
  
    //depth stream
    for(int y=0; y<RS_HEIGHT; ++y){
      for(int x=0; x<RS_WIDTH; ++x){
        int depth = *depth_frame++;
        dep_fra[y*RS_WIDTH + x] = depth * rs_get_device_depth_scale(dev, &e);
      }
    }
    uchar* colortemp = (unsigned char *) (rs_get_frame_data(rs_device_, RS_STREAM_COLOR, 0));
    cv::Mat(RS_HEIGHT, RS_WIDTH, CV_32FC1, dep_fra).copyTo(MatDepth);
    cv::Mat(RS_HEIGHT, RS_WIDTH, CV_8UC3, colortemp).copyTo(MatColor);
}
void run(){
  float *dep_fra = new float[RS_HEIGHT * RS_WIDTH];
  while(ros::ok()){
    if(waitKey(1) == 27) break;
    getStreamdata(dep_fra);
    setImageMsg();
    setCamInfoMsg();
    prepareTransforms();
    TopicPublisher();
  }
}
int main(int argc, char **argv){
  ros::init(argc, argv, "capture_image");
 
  ros::NodeHandle n;
  color_pub = n.advertise<sensor_msgs::Image>("/rs/color/image_raw", 5);  
  depth_pub = n.advertise<sensor_msgs::Image>("/rs/depth/image_raw", 5);
  cam_info_color_pub = n.advertise<sensor_msgs::CameraInfo>("/rs/color/camera_info", 5);  
  cam_info_depth_pub = n.advertise<sensor_msgs::CameraInfo>("/rs/depth/camera_info", 5);
 
  tfbr = new tf::TransformBroadcaster();

  //set camera
/* Create a context object. This object owns the handles to all connected realsense devices. */
  ctx = rs_create_context(RS_API_VERSION, &e);
  check_error();
  printf("There are %d connected RealSense devices.\n", rs_get_device_count(ctx, &e));
  check_error();
  if(rs_get_device_count(ctx, &e) == 0) return -1;

  /* This tutorial will access only a single device, but it is trivial to extend to multiple devices */
  dev = rs_get_device(ctx, 0, &e);
  check_error();
  printf("\nUsing device 0, an %s\n", rs_get_device_name(dev, &e));
  check_error();
  printf("    Serial number: %s\n", rs_get_device_serial(dev, &e));
  check_error();
  printf("    Firmware version: %s\n", rs_get_device_firmware_version(dev, &e));
  check_error();

  rs_device_ = rs_get_device(ctx, 0, &e);//
  /* Configure depth to run at VGA resolution at 30 frames per second */
  rs_enable_stream(dev, RS_STREAM_DEPTH, 640, 480, RS_FORMAT_Z16, 30, &e);
  check_error();
  rs_enable_stream(dev, RS_STREAM_COLOR, 640, 480, RS_FORMAT_RGB8, 60, &e);
  check_error();
 
  rs_start_device(dev, &e);
  check_error();


  run();//main loop

  return 0;
}
