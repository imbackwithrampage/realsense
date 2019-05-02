/*
 * Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 */

/*
 * Independent pixel point conversion ROS node. Can run this offline, and independent of having an actual
 * realsense attached.
 */

///////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                  //
///////////////////////////////////////////////////////////////////////////////

#include <realsense_pixel_point_conversion_service.h>
#include <supported_intrinsics.h>

#include <ros/ros.h>

#include <iostream>
#include <string>

///////////////////////////////////////////////////////////////////////////////
// CUSTOM STRUCTURES                                                         //
///////////////////////////////////////////////////////////////////////////////

/** Name of node. */
constexpr char node_name_[] = "PixelPointConversionRealsenseNode";

/** Parameter string for loading the camera profile using ros params. */
constexpr char param_camera_profile_[] = "pixel_point_conversion_service/profile";

/** Default camera profile. */
constexpr char default_camera_profile_[] = "D435_720P_ALIGNED";

///////////////////////////////////////////////////////////////////////////////
// STATIC FUNCTIONS                                                          //
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Get the intrinsic for the camera from a ros parameter, or use the default if not set.
 * @pararm nh ROS nodehandle.
 * @return Camera intrinsics for the camera profile.
 */
static const rs2_intrinsics getIntrinsics(ros::NodeHandle& nh)
{
  std::string profile;
  nh.param(std::string(param_camera_profile_), profile, std::string(default_camera_profile_));
  return realsense2_camera::getIntrinsics(profile);
}

///////////////////////////////////////////////////////////////////////////////
// MAIN FUNCTION                                                             //
///////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
  using namespace realsense2_camera;

  ros::init(argc, argv, node_name_);
  ros::NodeHandle nh;
  auto intrinsics = getIntrinsics(nh);
  RealsensePixelPointConversionService node(nh, intrinsics);  // RAII

  ros::spin();

  return 0;
}