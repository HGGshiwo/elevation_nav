#include "elevation_local_planner/velocity_smoother.h"

namespace elevation_local_planner
{

VelocitySmoother::VelocitySmoother()
: params_()
{
  reset();
}

VelocitySmoother::VelocitySmoother(const VelocitySmootherParams & params)
: params_(params)
{
  reset();
}

void VelocitySmoother::setParams(const VelocitySmootherParams & params)
{
  params_ = params;
}

void VelocitySmoother::reset()
{
  last_cmd_ = geometry_msgs::Twist();
  first_run_ = true;
}

geometry_msgs::Twist VelocitySmoother::smooth(const geometry_msgs::Twist & raw_target_cmd, double dt)
{
  geometry_msgs::Twist target = raw_target_cmd;

  // 1. 先进行幅值硬限幅
  target.linear.x = LocalControlUtils::clamp(target.linear.x, -params_.max_linear_speed, params_.max_linear_speed);
  target.linear.y = LocalControlUtils::clamp(target.linear.y, -params_.max_lateral_speed, params_.max_lateral_speed);
  target.angular.z = LocalControlUtils::clamp(target.angular.z, -params_.max_angular_speed, params_.max_angular_speed);

  // 2. 运动学解耦: 大角速度急转弯时自适应抑制侧向横移，防侧翻打滑
  if (params_.enable_lateral_decoupling && params_.max_angular_speed > 1.0e-3)
  {
    const double turn_ratio = LocalControlUtils::clamp(std::abs(target.angular.z) / params_.max_angular_speed, 0.0, 1.0);
    const double lateral_scale = std::max(0.0, 1.0 - turn_ratio * turn_ratio);
    target.linear.y *= lateral_scale;
  }

  if (first_run_ || dt <= 1.0e-6)
  {
    first_run_ = false;
    last_cmd_ = target;
    geometry_msgs::Twist out = target;
    out.linear.x = LocalControlUtils::applyDeadband(out.linear.x, params_.linear_deadband);
    out.linear.y = LocalControlUtils::applyDeadband(out.linear.y, params_.lateral_deadband);
    out.angular.z = LocalControlUtils::applyDeadband(out.angular.z, params_.angular_deadband);
    return out;
  }

  // 3. 动态加速度平滑滤波
  const double max_dv_x = std::max(0.01, params_.max_linear_acc * dt);
  const double max_dv_y = std::max(0.01, params_.max_lateral_acc * dt);
  const double max_dv_w = std::max(0.01, params_.max_angular_acc * dt);

  geometry_msgs::Twist smoothed;
  smoothed.linear.x = last_cmd_.linear.x + LocalControlUtils::clamp(target.linear.x - last_cmd_.linear.x, -max_dv_x, max_dv_x);
  smoothed.linear.y = last_cmd_.linear.y + LocalControlUtils::clamp(target.linear.y - last_cmd_.linear.y, -max_dv_y, max_dv_y);
  smoothed.angular.z = last_cmd_.angular.z + LocalControlUtils::clamp(target.angular.z - last_cmd_.angular.z, -max_dv_w, max_dv_w);

  smoothed.linear.x = LocalControlUtils::clamp(smoothed.linear.x, -params_.max_linear_speed, params_.max_linear_speed);
  smoothed.linear.y = LocalControlUtils::clamp(smoothed.linear.y, -params_.max_lateral_speed, params_.max_lateral_speed);
  smoothed.angular.z = LocalControlUtils::clamp(smoothed.angular.z, -params_.max_angular_speed, params_.max_angular_speed);

  last_cmd_ = smoothed;

  // 4. 死区滤波输出
  geometry_msgs::Twist output = smoothed;
  output.linear.x = LocalControlUtils::applyDeadband(output.linear.x, params_.linear_deadband);
  output.linear.y = LocalControlUtils::applyDeadband(output.linear.y, params_.lateral_deadband);
  output.angular.z = LocalControlUtils::applyDeadband(output.angular.z, params_.angular_deadband);

  return output;
}

} // namespace elevation_local_planner
