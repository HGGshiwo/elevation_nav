#pragma once

#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <algorithm>
#include <memory>

namespace elevation_planner
{

/**
 * @struct LocalElevationGrid
 * @brief 局部活动踏面高程切片网格 (2.5D Surface Patch)
 * 
 * 以机器人足底连通踏面为基准生成的局部单值高程地图。
 * 供 TEB 规划器以 O(1) 双线性插值直接查询真实空间高度 z，
 * 计算精确 3D 几何欧氏距离与坡度速度补偿，免去对全局流形大图的昂贵遍历。
 */
struct LocalElevationGrid
{
  std::string frame_id{"map"};
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.05};
  int width{0};   // 对应 cols (x 方向栅格数)
  int height{0};  // 对应 rows (y 方向栅格数)
  std::vector<float> data; // row-major: data[r * width + c]

  LocalElevationGrid() = default;

  LocalElevationGrid(int w, int h, double res, double ox, double oy, const std::string& frame = "map", float default_z = 0.0f)
    : frame_id(frame), origin_x(ox), origin_y(oy), resolution(res), width(w), height(h),
      data(w * h, default_z)
  {
  }

  inline bool toGrid(double x, double y, int& r, int& c) const
  {
    if (resolution <= 1e-6 || width <= 0 || height <= 0) return false;
    c = static_cast<int>(std::floor((x - origin_x) / resolution));
    r = static_cast<int>(std::floor((y - origin_y) / resolution));
    return (r >= 0 && r < height && c >= 0 && c < width);
  }

  inline float getZ(int r, int c) const
  {
    if (r < 0 || r >= height || c < 0 || c >= width || data.empty())
    {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return data[static_cast<size_t>(r * width + c)];
  }

  inline void setZ(int r, int c, float val)
  {
    if (r >= 0 && r < height && c >= 0 && c < width && !data.empty())
    {
      data[static_cast<size_t>(r * width + c)] = val;
    }
  }

  /**
   * @brief 双线性插值查询连续坐标 (x, y) 处的踏面高程 z
   * @param x 世界坐标 x
   * @param y 世界坐标 y
   * @param out_z 输出插值高程
   * @return true 成功查得有效高程; false 超出范围或无有效踏面
   */
  inline bool interpolateZ(double x, double y, double& out_z) const
  {
    if (resolution <= 1e-6 || width <= 0 || height <= 0 || data.empty()) return false;

    double c_f = (x - origin_x) / resolution - 0.5;
    double r_f = (y - origin_y) / resolution - 0.5;

    int c0 = static_cast<int>(std::floor(c_f));
    int r0 = static_cast<int>(std::floor(r_f));
    int c1 = c0 + 1;
    int r1 = r0 + 1;

    // 边界检查
    if (c0 < 0 || r0 < 0 || c1 >= width || r1 >= height)
    {
      int cr = std::max(0, std::min(width - 1, static_cast<int>(std::round(c_f))));
      int rr = std::max(0, std::min(height - 1, static_cast<int>(std::round(r_f))));
      float val = getZ(rr, cr);
      if (std::isnan(val)) return false;
      out_z = static_cast<double>(val);
      return true;
    }

    float z00 = getZ(r0, c0);
    float z01 = getZ(r0, c1);
    float z10 = getZ(r1, c0);
    float z11 = getZ(r1, c1);

    int valid_count = 0;
    double sum_z = 0.0;
    if (!std::isnan(z00)) { sum_z += z00; valid_count++; }
    if (!std::isnan(z01)) { sum_z += z01; valid_count++; }
    if (!std::isnan(z10)) { sum_z += z10; valid_count++; }
    if (!std::isnan(z11)) { sum_z += z11; valid_count++; }

    if (valid_count == 0) return false;
    if (valid_count < 4)
    {
      // 位于踏面边缘，采用有效邻点均值兜底
      out_z = sum_z / valid_count;
      return true;
    }

    double u = c_f - c0;
    double v = r_f - r0;
    out_z = (1.0 - u) * (1.0 - v) * z00 +
            u * (1.0 - v) * z01 +
            (1.0 - u) * v * z10 +
            u * v * z11;
    return true;
  }
};

using LocalElevationGridPtr = std::shared_ptr<LocalElevationGrid>;
using LocalElevationGridConstPtr = std::shared_ptr<const LocalElevationGrid>;

} // namespace elevation_planner
