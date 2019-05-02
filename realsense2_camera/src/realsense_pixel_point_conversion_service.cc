// INCLUDES ///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "realsense_pixel_point_conversion_service.h"
#include <librealsense2/rsutil.h>

#include <iostream>
#include <stdio.h>

// NAMESPACES /////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace realsense2_camera
{
// PUBLIC METHODS /////////////////////////////////////////////////////////////////////////////////////////////////////

void RealsensePixelPointConversionService::printIntrinsics(const rs2_intrinsics &intrinsics)
{
  using namespace std;
  cout << "-------------------------------------\n";
  cout << "RS2_INTRINSICS:\n";
  cout << "Width: " << intrinsics.width << "\n";
  cout << "Height: " << intrinsics.height << "\n";

  cout << "ppx: " << hexfloat << intrinsics.ppx << "\n";
  cout << "ppy: " << hexfloat << intrinsics.ppy << "\n";
  cout << "fx: " << hexfloat << intrinsics.fx << "\n";
  cout << "fy: " << hexfloat << intrinsics.fy << "\n";

  cout << "distortion model (enum): " << static_cast<int>(intrinsics.model) << "\n";

  for (int i = 0; i < 4; ++i)
    cout << "coeffs[" << i << "]: " << hexfloat << intrinsics.coeffs[i] << "\n";

  cout << "-------------------------------------\n";
}

RealsensePixelPointConversionService::RealsensePixelPointConversionService(ros::NodeHandle &nh,
                                                                           const rs2_intrinsics &intrinsics)
    : intrinsics_(intrinsics)
{
  printIntrinsics(intrinsics);

  pixel_to_point_service_ =
      nh.advertiseService(pixel_to_point_service_name_, &RealsensePixelPointConversionService::pixelToPointRos, this);

  point_to_pixel_service_ =
      nh.advertiseService(point_to_pixel_service_name_, &RealsensePixelPointConversionService::pointToPixelRos, this);
}

// PRIVATE METHODS ////////////////////////////////////////////////////////////////////////////////////////////////////

bool RealsensePixelPointConversionService::pixelToPointRos(realsense2_camera::PixelToPoint::Request &request,
                                                           realsense2_camera::PixelToPoint::Response &response)
{
  const float pixel[2]{request.x, request.y};

  float point[3];
  rs2_deproject_pixel_to_point(point, &intrinsics_, pixel, request.z);

  response.x = point[0];
  response.y = point[1];
  response.z = point[2];

  return true;
}

bool RealsensePixelPointConversionService::pointToPixelRos(realsense2_camera::PointToPixel::Request &request,
                                                           realsense2_camera::PointToPixel::Response &response)
{
  const float point[3]{request.x, request.y, request.z};

  float pixel[2];
  rs2_project_point_to_pixel(pixel, &intrinsics_, point);

  response.x = pixel[0];
  response.y = pixel[1];

  return true;
}

} // namespace realsense2_camera
