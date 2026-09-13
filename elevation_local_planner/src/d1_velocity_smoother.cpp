#include "elevation_local_planner/d1_velocity_smoother.h"

namespace elevation_local_planner
{

D1VelocitySmoother::D1VelocitySmoother()
: params_()
{
  reset();
}

D1VelocitySmoother::D1VelocitySmoother(const VelocitySmootherParams & params)
: params_(params)
{
  reset();
}

void D1VelocitySmoother::setParams(const VelocitySmootherParams & params)
{
  params_ = params;
}

void D1VelocitySmoother::reset()
{
  last_cmd_ = geometry_msgs::Twist();
  first_run_ = true;
}

geometry_msgs::Twist D1VelocitySmoother::smooth(const geometry_msgs::Twist & raw_target_cmd, double dt)
{
  geometry_msgs::Twist target = raw_target_cmd;

  // Clamp raw target limits first
  target.linear.x = D1ControlUtils::clamp(target.linear.x, -params_.max_linear_speed, params_.max_linear_speed);
  target.linear.y = D1ControlUtils::clamp(target.linear.y, -params_.max_lateral_speed, params_.max_lateral_speed);
  target.angular.z = D1ControlUtils::clamp(target.angular.z, -params_.max_angular_speed, params_.max_angular_speed);

  // 1. Motion Decoupling: Suppress lateral speed during sharp turns to prevent rollover.
  if (params_.enable_lateral_decoupling && params_.max_angular_speed > 1.0e-3)
  {
    const double turn_ratio = D1ControlUtils::clamp(std::abs(target.angular.z) / params_.max_angular_speed, 0.0, 1.0);
    const double lateral_scale = std::max(0.0, 1.0 - turn_ratio * turn_ratio);
    target.linear.y *= lateral_scale;
  }

  if (first_run_ || dt <= 1.0e-6)
  {
    first_run_ = false;
    last_cmd_ = target;
    geometry_msgs::Twist out = target;
    out.linear.x = D1ControlUtils::applyDeadband(out.linear.x, params_.linear_deadband);
    out.linear.y = D1ControlUtils::applyDeadband(out.linear.y, params_.lateral_deadband);
    out.angular.z = D1ControlUtils::applyDeadband(out.angular.z, params_.angular_deadband);
    return out;
  }

  // 2. Acceleration Limit Filtering
  const double max_dv_x = std::max(0.01, params_.max_linear_acc * dt);
  const double max_dv_y = std::max(0.01, params_.max_lateral_acc * dt);
  const double max_dv_w = std::max(0.01, params_.max_angular_acc * dt);

  geometry_msgs::Twist smoothed;
  smoothed.linear.x = last_cmd_.linear.x + D1ControlUtils::clamp(target.linear.x - last_cmd_.linear.x, -max_dv_x, max_dv_x);
  smoothed.linear.y = last_cmd_.linear.y + D1ControlUtils::clamp(target.linear.y - last_cmd_.linear.y, -max_dv_y, max_dv_y);
  smoothed.angular.z = last_cmd_.angular.z + D1ControlUtils::clamp(target.angular.z - last_cmd_.angular.z, -max_dv_w, max_dv_w);

  smoothed.linear.x = D1ControlUtils::clamp(smoothed.linear.x, -params_.max_linear_speed, params_.max_linear_speed);
  smoothed.linear.y = D1ControlUtils::clamp(smoothed.linear.y, -params_.max_lateral_speed, params_.max_lateral_speed);
  smoothed.angular.z = D1ControlUtils::clamp(smoothed.angular.z, -params_.max_angular_speed, params_.max_angular_speed);

  // Maintain un-corrupted acceleration integration state in last_cmd_
  last_cmd_ = smoothed;

  // 3. Apply Deadbands to output
  geometry_msgs::Twist output = smoothed;
  output.linear.x = D1ControlUtils::applyDeadband(output.linear.x, params_.linear_deadband);
  output.linear.y = D1ControlUtils::applyDeadband(output.linear.y, params_.lateral_deadband);
  output.angular.z = D1ControlUtils::applyDeadband(output.angular.z, params_.angular_deadband);

  return output;
}

} // namespace elevation_local_planner
