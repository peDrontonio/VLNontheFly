#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "plan_env/depth_projection.h"

namespace
{

constexpr double kTolerance = 1e-9;

TEST(DepthProjection, UsesCurrentPixelForInvalidDepth)
{
  const std::vector<uint16_t> raw_depths{0, 1000, 0, 2000, 0, 3000};
  const std::vector<double> expected{4.1, 1.0, 4.1, 2.0, 4.1, 3.0};

  for (std::size_t i = 0; i < raw_depths.size(); ++i)
  {
    const auto depth = plan_env::depthForRaycast(raw_depths[i], 0.001, 0.2, 4.0, 4.0);
    ASSERT_TRUE(depth.has_value());
    EXPECT_NEAR(*depth, expected[i], kTolerance);
  }
}

TEST(DepthProjection, RejectsCurrentPixelInsideMinimumRange)
{
  EXPECT_FALSE(plan_env::depthForRaycast(100, 0.001, 0.2, 4.0, 4.0).has_value());
}

TEST(DepthProjection, AppliesFullOpticalToTargetTransform)
{
  constexpr double pitch = 7.0 * M_PI / 180.0;
  const double c = std::cos(pitch);
  const double s = std::sin(pitch);

  Eigen::Matrix3d base_from_optical;
  base_from_optical << 0.0, -s, c,
                       -1.0, 0.0, 0.0,
                       0.0, -c, -s;

  const plan_env::PinholeIntrinsics intrinsics{100.0, 100.0, 50.0, 40.0};
  const Eigen::Vector3d translation(0.155, 0.0, 0.0);

  const Eigen::Vector3d front = plan_env::projectPixel(
      50, 40, 1.0, intrinsics, base_from_optical, translation);
  EXPECT_NEAR(front.x(), 0.155 + c, kTolerance);
  EXPECT_NEAR(front.y(), 0.0, kTolerance);
  EXPECT_NEAR(front.z(), -s, kTolerance);

  const Eigen::Vector3d image_right = plan_env::projectPixel(
      150, 40, 1.0, intrinsics, base_from_optical, translation);
  EXPECT_LT(image_right.y(), 0.0);

  const Eigen::Vector3d image_below = plan_env::projectPixel(
      50, 140, 1.0, intrinsics, base_from_optical, translation);
  EXPECT_LT(image_below.z(), front.z());
}

TEST(DepthProjection, UsesLiveImageDimensionsForCapacity)
{
  EXPECT_EQ(plan_env::projectionCapacity(480, 640, 2), 76800U);
  EXPECT_EQ(plan_env::projectionCapacity(480, 848, 2), 101760U);
  EXPECT_EQ(plan_env::projectionCapacity(3, 5, 2), 6U);
}

TEST(DepthProjection, LooksUpCameraPoseAtDepthTimestamp)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);

  geometry_msgs::msg::TransformStamped base_from_depth;
  base_from_depth.header.frame_id = "base_link";
  base_from_depth.child_frame_id = "depth_optical_frame";
  base_from_depth.transform.translation.x = 0.155;
  base_from_depth.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(base_from_depth, "test", true));

  geometry_msgs::msg::TransformStamped map_from_base;
  map_from_base.header.frame_id = "map";
  map_from_base.child_frame_id = "base_link";
  map_from_base.header.stamp.sec = 10;
  map_from_base.transform.translation.x = 1.0;
  map_from_base.transform.rotation.w = 1.0;
  ASSERT_TRUE(buffer.setTransform(map_from_base, "test"));

  map_from_base.header.stamp.sec = 11;
  map_from_base.transform.translation.x = 5.0;
  ASSERT_TRUE(buffer.setTransform(map_from_base, "test"));

  const auto map_from_depth = buffer.lookupTransform(
      "map", "depth_optical_frame", rclcpp::Time(10, 0, RCL_SYSTEM_TIME));
  EXPECT_NEAR(map_from_depth.transform.translation.x, 1.155, kTolerance);
}

}  // namespace
