// INCLUDES ///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "RealsensePixelPointConversionService.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>

// DEFINES ////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* This is NECESSARY if you are linking against rostest_gmock_main. */
char g_nodename[] = "RealsensePixelPointConversionServiceTestNode";

/*
 * Format we'll use to store request/responses.
 */
template<typename T>
struct PixelPoint
{
  PixelPoint() : x(0), y(0), z(0) {}

  PixelPoint(const T a, const T b, const T c) : x(a), y(b), z(c)
  {
  }

  T x;
  T y;
  T z;
};

template<typename T>
bool operator==(const PixelPoint<T>& P, const PixelPoint<T>& Q)
{
  const double epsilon = 0.001;
  double sum = 0;
  sum += pow(P.x - Q.x, 2);
  sum += pow(P.y - Q.y, 2);
  sum += pow(P.z - Q.z, 2);

  sum = sqrt(sum);

  return sum < epsilon;
}

template<typename T>
bool operator!=(const PixelPoint<T>& P, const PixelPoint<T>& Q)
{
  return !(P==Q);
}

// FIXTURES ///////////////////////////////////////////////////////////////////////////////////////////////////////////

class ServiceTest : public ::testing::Test
{
protected:
  ServiceTest() {}

  void SetUp() override
  {
    loadParams();
    initTestNode();
  }

  void TearDown() override
  {
  }

  void loadParams()
  {
  }

  void initTestNode()
  {
    using namespace realsense2_camera;

    /* Setup service clients. */
    pixel_to_point_client = nh.serviceClient<realsense2_camera::PixelToPoint>(pixel_to_point_service_name_);

    point_to_pixel_client = nh.serviceClient<realsense2_camera::PointToPixel>(point_to_pixel_service_name_);
  }

  template<typename T>
  bool pixelPointCall(const PixelPoint<T> request)
  {
    using namespace realsense2_camera;

    /* Check service is ready. */
    const bool exists = pixel_to_point_client.waitForExistence(ros::Duration(1));
    if (!exists)
    {
      std::cerr << "Pixel to point service doesn't exist\n";
      return false;
    }

    /* Make the call. */
    PixelToPoint srv;
    srv.request.x = request.x;
    srv.request.y = request.y;
    srv.request.z = request.z;

    if (!pixel_to_point_client.call(srv))
    {
      std::cerr << "Client call failed.\n";
      return false;
    }

    /* Save reply. */
    response.x = srv.response.x;
    response.y = srv.response.y;
    response.z = srv.response.z;

    return true;
  }

  template<typename T>
  bool pointPixelCall(const PixelPoint<T> request)
  {
    using namespace realsense2_camera;

    /* Check service is ready. */
    const bool exists = point_to_pixel_client.waitForExistence(ros::Duration(1));
    if (!exists)
    {
      std::cerr << "Point to pixel service doesn't exist\n";
      return false;
    }

    /* Make the call. */
    PointToPixel srv;
    srv.request.x = request.x;
    srv.request.y = request.y;
    srv.request.z = request.z;

    if (!point_to_pixel_client.call(srv))
    {
      std::cerr << "Client call failed.\n";
      return false;
    }

    /* Save reply. */
    response.x = srv.response.x;
    response.y = srv.response.y;
    response.z = request.z;

    return true;
  }

  /* ROS contexts. */
  ros::NodeHandle nh;
  ros::ServiceClient pixel_to_point_client;
  ros::ServiceClient point_to_pixel_client;

  /* Pixel point contexts. */
  // const PixelPoint<double> pixel_truth;
  // const PixelPoint<double> point_truth;

  PixelPoint<double> response;
};

// TESTS //////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Note these tests require realsense to spin up successfully. Otherwise it will fail.
 */

/* Test pixel to point service. */
TEST_F(ServiceTest, PixelToPointServiceCall)
{
  const PixelPoint<double> pixel_truth(400,600,3);
  const PixelPoint<double> point_truth(0.363676,1.74069,3);

  EXPECT_TRUE(pixelPointCall(pixel_truth));
  EXPECT_TRUE(point_truth == response);
}

/* Test point to pixel service. */
TEST_F(ServiceTest, PointToPixelServiceCall)
{
  const PixelPoint<double> pixel_truth(400,600,3);
  const PixelPoint<double> point_truth(0.363676,1.74069,3);

  EXPECT_TRUE(pointPixelCall(point_truth));
  EXPECT_TRUE(pixel_truth == response);
}