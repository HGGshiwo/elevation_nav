#ifndef ELEVATION_LOCAL_PLANNER_VELOCITY_SMOOTHER_H_
#define ELEVATION_LOCAL_PLANNER_VELOCITY_SMOOTHER_H_

#include <geometry_msgs/Twist.h>
#include "elevation_local_planner/control_types.h"

namespace elevation_local_planner
{

struct VelocitySmootherParams
{
  double max_linear_speed{0.60};
  double max_lateral_speed{0.30};
  double max_angular_speed{1.20};

  double max_linear_acc{0.80};   // m/s^2
  double max_lateral_acc{0.40};  // m/s^2
  double max_angular_acc{1.20};  // rad/s^2

  bool enable_lateral_decoupling{true};

  double linear_deadband{0.05};
  double lateral_deadband{0.05};
  double angular_deadband{0.05};
};

class VelocitySmoother
{
public:
  VelocitySmoother();
  explicit VelocitySmoother(const VelocitySmootherParams & params);

  void setParams(const VelocitySmootherParams & params);
  const VelocitySmootherParams & getParams() const { return params_; }

  void reset();

  geometry_msgs::Twist smooth(const geometry_msgs::Twist & raw_target_cmd, double dt);

  const geometry_msgs::Twist & getLastCmd() const { return last_cmd_; }

private:
  VelocitySmootherParams params_;
  geometry_msgs::Twist last_cmd_;
  bool first_run_{true};
};

} // namespace elevation_local_planner

#endif // ELEVATION_LOCAL_PLANNER_VELOCITY_SMOOTHER_H_
