#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>

#include "elevation_local_planner/elevation_local_planner.hpp"

class ElevationLocalPlannerNode
{
public:
  ElevationLocalPlannerNode(ros::NodeHandle & nh, ros::NodeHandle & pnh)
    : tf_listener_(tf_buffer_)
  {
    pnh.param<std::string>("map_frame", map_frame_, "map");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
    pnh.param<double>("controller_freq", controller_freq_, 20.0);

    elevation_local_planner::LocalPlannerConfig cfg;
    pnh.param<double>("max_vel_x", cfg.max_vel_x, 0.8);
    pnh.param<double>("max_vel_theta", cfg.max_vel_theta, 1.0);
    pnh.param<double>("lookahead_dist", cfg.lookahead_dist, 0.8);
    planner_.setConfig(cfg);

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
    path_sub_ = nh.subscribe("/elevation_global_plan", 1, &ElevationLocalPlannerNode::onPath, this);

    timer_ = nh.createTimer(ros::Duration(1.0 / controller_freq_), &ElevationLocalPlannerNode::onControlLoop, this);
    ROS_INFO("[ElevationLocalPlannerNode] Local trajectory tracker started (%.1f Hz)", controller_freq_);
  }

  void onPath(const nav_msgs::Path::ConstPtr & path_msg)
  {
    planner_.setPlan(*path_msg);
    has_path_ = true;
    ROS_INFO("[ElevationLocalPlannerNode] Received new global path, start tracking");
  }

  void onControlLoop(const ros::TimerEvent &)
  {
    if (!has_path_) return;

    geometry_msgs::PoseStamped cur_pose;
    cur_pose.header.frame_id = map_frame_;
    cur_pose.header.stamp = ros::Time::now();

    try {
      geometry_msgs::TransformStamped tf_stamped =
        tf_buffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0), ros::Duration(0.1));
      cur_pose.pose.position.x = tf_stamped.transform.translation.x;
      cur_pose.pose.position.y = tf_stamped.transform.translation.y;
      cur_pose.pose.position.z = tf_stamped.transform.translation.z;
      cur_pose.pose.orientation = tf_stamped.transform.rotation;
    } catch (const tf2::TransformException & ex) {
      return;
    }

    geometry_msgs::Twist cmd_vel;
    if (planner_.computeVelocityCommands(cur_pose, cmd_vel)) {
      cmd_vel_pub_.publish(cmd_vel);
      if (planner_.isGoalReached()) {
        ROS_INFO("[ElevationLocalPlannerNode] Goal reached!");
        has_path_ = false;
        geometry_msgs::Twist stop_cmd;
        cmd_vel_pub_.publish(stop_cmd);
      }
    }
  }

private:
  std::string map_frame_;
  std::string base_frame_;
  std::string cmd_vel_topic_;
  double controller_freq_{20.0};
  bool has_path_{false};

  elevation_local_planner::ElevationLocalPlanner planner_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber path_sub_;
  ros::Timer timer_;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "elevation_local_planner_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ElevationLocalPlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
