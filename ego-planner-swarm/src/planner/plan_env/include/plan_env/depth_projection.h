#ifndef PLAN_ENV__DEPTH_PROJECTION_H_
#define PLAN_ENV__DEPTH_PROJECTION_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <Eigen/Core>

namespace plan_env
{

struct PinholeIntrinsics
{
  double fx;
  double fy;
  double cx;
  double cy;
};

inline std::optional<double> depthForRaycast(
    uint16_t raw_depth, double inverse_scale, double min_depth,
    double max_depth, double max_ray_length)
{
  if (raw_depth == 0)
  {
    return max_ray_length + 0.1;
  }

  const double depth = static_cast<double>(raw_depth) * inverse_scale;
  if (depth < min_depth)
  {
    return std::nullopt;
  }
  if (depth > max_depth)
  {
    return max_ray_length + 0.1;
  }
  return depth;
}

inline Eigen::Vector3d projectPixel(
    int u, int v, double depth, const PinholeIntrinsics & intrinsics,
    const Eigen::Matrix3d & target_rotation,
    const Eigen::Vector3d & target_translation)
{
  const Eigen::Vector3d point_optical(
      (static_cast<double>(u) - intrinsics.cx) * depth / intrinsics.fx,
      (static_cast<double>(v) - intrinsics.cy) * depth / intrinsics.fy,
      depth);
  return target_rotation * point_optical + target_translation;
}

inline std::size_t projectionCapacity(int rows, int cols, int skip_pixel)
{
  const int safe_skip = std::max(skip_pixel, 1);
  const std::size_t sampled_rows =
      static_cast<std::size_t>((std::max(rows, 0) + safe_skip - 1) / safe_skip);
  const std::size_t sampled_cols =
      static_cast<std::size_t>((std::max(cols, 0) + safe_skip - 1) / safe_skip);
  return sampled_rows * sampled_cols;
}

}  // namespace plan_env

#endif  // PLAN_ENV__DEPTH_PROJECTION_H_
