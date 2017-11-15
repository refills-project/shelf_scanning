// This node listens to the ROS image message topic "barcode/image",
// converts the sensor_msgs::Image to a HalconCpp::HImage,
// process the HalconCpp::HImage, publishes the barcode number and location on
// the " " topic. The image is then republished over ROS.

#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <refills_msgs/Barcode.h>
#include <halcon_image.h>

// Using image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>

#ifndef __APPLE__
#  include "HalconCpp.h"
#  include "HDevThread.h"
#  if defined(__linux__) && !defined(NO_EXPORT_APP_MAIN)
#    include <X11/Xlib.h>
#  endif
#else
#  ifndef HC_LARGE_IMAGES
#    include <HALCONCpp/HalconCpp.h>
#    include <HALCONCpp/HDevThread.h>
#  else
#    include <HALCONCppxl/HalconCpp.h>
#    include <HALCONCppxl/HDevThread.h>
#  endif
#  include <stdio.h>
#  include <HALCON/HpThread.h>
#  include <CoreFoundation/CFRunLoop.h>
#endif

using namespace HalconCpp;

class ImageConverter{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher barcode_pub_;
  
  // Local iconic variables
  HObject  ho_Image, ho_SymReg;
   
  // Local control variables
  HTuple  hv_BHandle, hv_Dec;
  HTuple  hv_Area, hv_Row, hv_Column;
  
  // Halcon image to process
  HImage *halcon_msg_image;
  sensor_msgs::Image image_msg;
  halcon_bridge::HalconImage hbridge_img;
  HImage display_image;





public:
  ImageConverter()
    : it_(nh_){
    // Subscribe and  publish using image_transport
    image_sub_ = it_.subscribe("barcode/input", 1, &ImageConverter::imageCb, this); // barcode/image
    image_pub_ = it_.advertise("barcode/output", 1);
    barcode_pub_ = nh_.advertise<refills_msgs::Barcode>("barcode/pose", 100);
  }

  ~ImageConverter()
  {
    //
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg){
    halcon_bridge::HalconImagePtr halcon_ptr;
    bool found_barcodes = false;

//    std::cout << "---imageCb---" << std::endl; 

    try{
      halcon_ptr = halcon_bridge::toHalconCopy(msg);
    }
      catch (halcon_bridge::Exception& e){
      ROS_ERROR("halcon_bridge exception: %s", e.what());
      return;
    }



    // Process halcon_ptr->image using Halcon
    halcon_msg_image = halcon_ptr->image;


    //FindBarCode needs a single-channel image (gray values)
    HImage gray_image = halcon_msg_image->Rgb1ToGray();
    HBarCode bc;
    bc.CreateBarCodeModel(HTuple(), HTuple());

    HString found_bc_string;
    HTuple found_bc_strings;

    HRegion bc_regions;
    int num_barcodes = 0;

    try {
      bc_regions = gray_image.FindBarCode(bc, "EAN-8", &found_bc_strings);
      num_barcodes = found_bc_strings.Length();

    } catch (HException &except) {
      std::cout << "Exception while running FindBarCode" << std::endl;
      std::cout << except.ErrorMessage() << std::endl;
    }

    if (num_barcodes == 0) {
      try {
        hbridge_img.image = halcon_msg_image;
        hbridge_img.header.frame_id = halcon_ptr->header.frame_id;
        hbridge_img.header.stamp = halcon_ptr->header.stamp;
        hbridge_img.encoding = halcon_ptr->encoding;

        hbridge_img.toImageMsg(image_msg);
        image_pub_.publish(image_msg);
      } catch (HException &except) {
        std::cout << "Exception while publishing the result image" << std::endl;
        std::cout << except.ErrorMessage() << std::endl;
      }


      return;
    }


//    std::cout << "Found # of barcodes: " << num_barcodes << std::endl;

    //std::cout << "Barcodes found: " << found_bc_strings.ToString() << std::endl;

    for (unsigned int i=0; i < num_barcodes ; ++i) {
      HString code = found_bc_strings[i];
//      std::cout << "Barcode: " << code << std::endl;
      refills_msgs::Barcode barcode_msg;

      barcode_msg.barcode_pose.header.frame_id = halcon_ptr->header.frame_id;
      barcode_msg.barcode_pose.header.stamp = halcon_ptr->header.stamp;

      barcode_msg.barcode_pose.pose.orientation.w = 1;
      barcode_msg.barcode_pose.pose.position.z = 0.2;

      barcode_msg.barcode = code;
      barcode_pub_.publish(barcode_msg);
    }

    try {

      HTuple grayvalue(120.0, 120.0);
      //grayvalue.Append(HTuple(0.0));

      //paint on top of the original color image
      display_image = halcon_msg_image->PaintRegion(bc_regions, grayvalue, HString("fill"));
      //display_image = gray_image.PaintRegion(bc_regions, grayvalue, HString("fill"));


    } catch (HException &except) {
      std::cout << "Exception while running PaintRegion" << std::endl;
      std::cout << except.ErrorMessage() << std::endl;
    }


    try {
      hbridge_img.image = &display_image;
      hbridge_img.header.frame_id = halcon_ptr->header.frame_id;
      hbridge_img.header.stamp = halcon_ptr->header.stamp;
      hbridge_img.encoding = halcon_ptr->encoding;

      hbridge_img.toImageMsg(image_msg);
      image_pub_.publish(image_msg);
    } catch (HException &except) {
      std::cout << "Exception while publishing the result image" << std::endl;
      std::cout << except.ErrorMessage() << std::endl;
    }


   // std::cout << "***" << std::endl;



  }
};

int main(int argc, char **argv)
{
  std::cout << "Starting the barcode detector." << std::endl;
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  std::cout << "Stopping the barcode detector." << std::endl;

  return 0;
}
