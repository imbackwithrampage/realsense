/*
 * ROS service for converting 2D pixel-depth coordinates into a 3D point coordinate.
 * Uses the RGB camera intrinsics.
 * For pixel -> point it uses rs2_deproject_pixel_to_point() from librealsense2.
 * For point -> pixel it uses rs2_project_point_to_pixel() from librealsense2.
 */

#ifndef HANSONROBOTICS_PIXEL_POINT_CONVERSION_SERVICE_H_
#define HANSONROBOTICS_PIXEL_POINT_CONVERSION_SERVICE_H_

// INCLUDES ///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <realsense2_camera/PixelToPoint.h>
#include <realsense2_camera/PointToPixel.h>

#include <librealsense2/h/rs_types.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <string>

// NAMESPACES /////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace realsense2_camera
{

// STRUCTS ////////////////////////////////////////////////////////////////////////////////////////////////////////////

/** Name of the pixel to point service. */
constexpr char pixel_to_point_service_name_[] = "/color/pixel_to_point";

/** Name of the point to pixel service. */
constexpr char point_to_pixel_service_name_[] = "/color/point_to_pixel";

// CLASSES ////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Service class for handling point to pixel and pixel to point coordinate conversion requests for the
 * Intel realsense.
 */
class RealsensePixelPointConversionService final
{
public:
  /**
   * @param nh Node handle for the ROS node or nodelet.
   * @param intrinsics Intrinsics information for the camera.
   */
  explicit RealsensePixelPointConversionService(ros::NodeHandle &nh, const rs2_intrinsics &intrinsics);

#ifndef HR_DEBUG
private:
#endif
  /** Camera intrinsics. */
  rs2_intrinsics intrinsics_;

  /** Pixel to point service object. */
  ros::ServiceServer pixel_to_point_service_;

  /** Point to pixel service object. */
  ros::ServiceServer point_to_pixel_service_;

  /**
   * @brief Callback function for handling pixel to point calls from ROS.
   * @param request Request values. Defined in the PixelToPoint.srv message file.
   * @param response Response values. Defined in the PixelToPoint.srv message file.
   * @return Always returning true for now.
   */
  bool pixelToPointRos(realsense2_camera::PixelToPoint::Request &request,
                       realsense2_camera::PixelToPoint::Response &response);

  /**
   * @brief Callback function for handling point to pixel calls from ROS.
   * @param request Request values. Defined in the PointToPixel.srv message file.
   * @param response Response values.Defined in the PointToPixel.srv message file.
   * @return Always returning true for now.
   */
  bool pointToPixelRos(realsense2_camera::PointToPixel::Request &request,
                       realsense2_camera::PointToPixel::Response &response);
};

} // namespace realsense2_camera

#endif // HANSONROBOTICS_PIXEL_POINT_CONVERSION_SERVICE_H_