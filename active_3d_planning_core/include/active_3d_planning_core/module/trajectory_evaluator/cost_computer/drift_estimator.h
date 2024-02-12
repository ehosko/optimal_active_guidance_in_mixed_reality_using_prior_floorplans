#ifndef ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_
#define ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_

#include "active_3d_planning_core/module/trajectory_evaluator.h"

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

namespace active_3d_planning {
namespace cost_computer {

// Execution time of a single segment + drift estimation of the segment
class DriftEstimator : public CostComputer {
 public:
  explicit DriftEstimator(PlannerI& planner);  // NOLINT

  bool computeCost(TrajectorySegment* traj_in) override;

  void setupFromParamMap(Module::ParamMap* param_map) override;

  std::tuple<Eigen::Vector3d, Eigen::Quaterniond> getCurrentPose(const std::string source_frame, const std::string target_frame);

  double computeDriftError(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation);
  double computeDriftErrorFloorplan(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation);
  double computeOptTrajError(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation);


 protected:
  static ModuleFactoryRegistry::Registration<DriftEstimator> registration;

  void projectOnFloor(Eigen::Vector3d pos, Eigen::Quaterniond q, Eigen::Vector3d& projVec);

  // params
  bool p_accumulate_;  // True: Use total time
  double drift_weight_;  // Weighting of the segment length vs. the drift
  double orientation_error_weight_; // Weighting of the orientation vs. the translation error

  bool use_opt_traj_;
  bool use_floorplan_asGT;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string source_frame_;
  std::string gt_frame_;
  std::string floorplan_frame_;
  std::string drifty_frame_;
  std::string opt_traj_frame;
  double drift_estimation_radius_;
  double drift_discount_;
};

}  // namespace cost_computer
}  // namespace active_3d_planning

#endif  // ACTIVE_3D_PLANNING_CORE_MODULE_TRAJECTORY_EVALUATOR_COST_COMPUTER_SEGMENT_TIME_H_
