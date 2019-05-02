/*
 * Copyright 2019 Hanson Robotics Limited. All Rights Reserved.
 */

/*
 * List of supported intrinsics we want to use for an offline pixel point conversion service.
 */

#ifndef HR_REALSENSE2_CAMERA_SUPPORTED_INTRINSICS_H_
#define HR_REALSENSE2_CAMERA_SUPPORTED_INTRINSICS_H_

///////////////////////////////////////////////////////////////////////////////
// INCLUDES                                                                  //
///////////////////////////////////////////////////////////////////////////////

#include <librealsense2/h/rs_types.h>

///////////////////////////////////////////////////////////////////////////////
// NAMESPACES                                                                //
///////////////////////////////////////////////////////////////////////////////

namespace realsense2_camera
{
///////////////////////////////////////////////////////////////////////////////
// CUSTOM STRUCTURES                                                         //
///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Camera intrinsics for the profile when we have 1920x1080 colour, where depth is aligned the
 * colour image.
 */
const rs2_intrinsics i_d435_1080p_aligned_ = { .width = 1920,
                                               .height = 1080,
                                               .ppx = 0x1.e612b2p+9,
                                               .ppy = 0x1.114828p+9,
                                               .fx = 0x1.5a293p+10,
                                               .fy = 0x1.5a2be2p+10,
                                               .model = RS2_DISTORTION_BROWN_CONRADY,
                                               .coeffs = { 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0 } };

/**
 * @brief Camera intrinsics for the profile when we have 1280x720 in colour, where depth is aligned the
 * colour image.
 */
const rs2_intrinsics i_d435_720p_aligned_ = { .width = 1280,
                                              .height = 720,
                                              .ppx = 0x1.440c78p+9,
                                              .ppy = 0x1.6c6036p+8,
                                              .fx = 0x1.cd8c4p+9,
                                              .fy = 0x1.cd8fdap+9,
                                              .model = RS2_DISTORTION_BROWN_CONRADY,
                                              .coeffs = { 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0 } };

/**
 * @brief Camera intrinsics for the profile when we have 640x480 in colour, where depth is aligned the
 * colour image.
 */
const rs2_intrinsics i_d435_480p_aligned_ = { .width = 640,
                                              .height = 480,
                                              .ppx = 0x1.4565f4p+8,
                                              .ppy = 0x1.e5d59cp+7,
                                              .fx = 0x1.33b2d6p+9,
                                              .fy = 0x1.33b53cp+9,
                                              .model = RS2_DISTORTION_BROWN_CONRADY,
                                              .coeffs = { 0x0p+0, 0x0p+0, 0x0p+0, 0x0p+0 } };

/*
 * Supported strings:
 * D435_1080P_ALIGNED - For a D435, 1920x1080 colour and depth, with depth aligned to colour.
 * D435_720P_ALIGNED - For a D435, 1280x720 colour and depth, with depth aligned to colour.
 * D435_480P_ALIGNED - For a D435, 640x480 colour and depth, with depth aligned to colour.
 *
 * Add others later if requirements come up.
 */

/**
 * @brief Get the camera intrinsics structure for a given profile.
 * @tparam T You don't need to specify this explicitly. Used for C++ forwarding here.
 * @param name Name of the profile we want intrinsics for.
 * @return Camera intrinsics for the given profile.
 * @throw Invalid argument exception if you specified an uncovered name.
 */
template <typename T>
const rs2_intrinsics getIntrinsics(T&& name)
{
  if (name == "D435_1080P_ALIGNED")
    return i_d435_1080p_aligned_;
  else if (name == "D435_720P_ALIGNED")
    return i_d435_720p_aligned_;
  else if (name == "D435_480P_ALIGNED")
    return i_d435_480p_aligned_;

  throw std::invalid_argument("getIntrinsics: unsupported profile requested.");
}

}  // namespace realsense2_camera

#endif  // HR_REALSENSE2_CAMERA_SUPPORTED_INTRINSICS_H_