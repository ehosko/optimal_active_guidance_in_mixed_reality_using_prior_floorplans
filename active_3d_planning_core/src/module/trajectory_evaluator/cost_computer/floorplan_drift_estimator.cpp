#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/floorplan_drift_estimator.h"

namespace active_3d_planning {
namespace cost_computer {

// SegmentTime
ModuleFactoryRegistry::Registration<FloorplanDriftEstimator> FloorplanDriftEstimator::registration(
    "FloorplanDriftEstimator");

FloorplanDriftEstimator::FloorplanDriftEstimator(PlannerI& planner) : CostComputer(planner), tf_listener_(tf_buffer_) {}

void FloorplanDriftEstimator::setupFromParamMap(Module::ParamMap* param_map) {
  // Needed in any case, want to accumulate from parents
  setParam<bool>(param_map, "accumulate", &p_accumulate_, true);
  setParam<double>(param_map, "drift_weight", &drift_weight_, 1);
  setParam<double>(param_map, "orientation_error_weight", &orientation_error_weight_, 10);
  setParam<std::string>(param_map, "world_frame", &source_frame_, "world");
  setParam<std::string>(param_map, "drifty_frame", &drifty_frame_, "rovioli/imu");
  setParam<std::string>(param_map, "floorplan_frame", &floorplan_frame_, "floorplan");
  setParam<double>(param_map, "drift_estimation_radius", &drift_estimation_radius_, 0.5);
  setParam<double>(param_map, "drift_discount", &drift_discount_, 0.9);
}

// TODO: (michbaum) This is the current cost computation used
bool FloorplanDriftEstimator::computeCost(TrajectorySegment* traj_in) {
  // Too short?
  if (traj_in->trajectory.size() < 2) {
    traj_in->cost = 0.0;
    return false;
  }

  double segment_time_cost = static_cast<double>(traj_in->trajectory.back().time_from_start_ns -
                                        traj_in->trajectory.front().time_from_start_ns) * 1.0e-9;

  std::tuple<Eigen::Vector3d, Eigen::Quaterniond> drifty_pose = getCurrentPose(source_frame_, drifty_frame_);
  Eigen::Vector3d drifty_position = std::get<0>(drifty_pose);
  Eigen::Quaterniond drifty_orientation = std::get<1>(drifty_pose);
  Eigen::Vector3d current_position_front = traj_in->trajectory.front().position_W;
  Eigen::Quaterniond current_orientation_front = traj_in->trajectory.front().orientation_W_B;
  Eigen::Vector3d current_position_back = traj_in->trajectory.back().position_W;
  Eigen::Quaterniond current_orientation_back = traj_in->trajectory.back().orientation_W_B;

  // Check if the trajectory start is close to our current drifty position -> Then we can add our drift estimate
  const double position_difference_start = (current_position_front - drifty_position).norm();
  const double position_difference_end = (current_position_back - drifty_position).norm();
  std::stringstream ss;
  // ss << "Position difference of this trajectory segment's start and the current position is:\n"
  //    << position_difference_start << "\n"
  //    << "Position difference of this trajectory segment's end and the current position is:\n"
  //    << position_difference_end << "\n";
  // std::cout << "Info: " << ss.str() << std::endl;

  if (position_difference_start > drift_estimation_radius_) {
    // If no valid drift estimate possible, just return the SegmentTime cost
    traj_in->cost = segment_time_cost;
    if (p_accumulate_ && traj_in->parent) {
    // TODO: Check that this really happens - it should!
      traj_in->cost += traj_in->parent->cost;
    }
    return true;
  }

  ss << "Estimating drift for the cost function!\n";
  std::cout << "Info: " << ss.str() << std::endl;

  ss << "Position difference of this trajectory segment's start and the current position is:\n"
     << position_difference_start << "\n"
     << "Position difference of this trajectory segment's end and the current position is:\n"
     << position_difference_end << "\n";
  std::cout << "Info: " << ss.str() << std::endl;

  // Extract the current ground truth pose
  std::tuple<Eigen::Vector3d, Eigen::Quaterniond> floorplan_pose = getCurrentPose(source_frame_, floorplan_frame_);
  Eigen::Vector3d gt_position = std::get<0>(floorplan_pose);
  //Eigen::Quaterniond gt_orientation = std::get<1>(gt_pose);

  double drift_cost = computeDriftError(gt_position, drifty_position);

  traj_in->cost = segment_time_cost + drift_weight_ * drift_cost;
      
  if (p_accumulate_ && traj_in->parent) {
    // TODO: Check that this really happens - it should!
    traj_in->cost += traj_in->parent->cost;
    // traj_in->cost += traj_in->parent->cost * drift_discount_;
  }
  return true;
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> FloorplanDriftEstimator::getCurrentPose(const std::string source_frame, const std::string target_frame){
  // Extract the transform from /world to the target frame at this point in time
  try
    {
      // Lookup the transform at the time of the pose message
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(target_frame, source_frame, ::ros::Time(0), ::ros::Duration(1.0));

      Eigen::Vector3d position = Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
      Eigen::Quaterniond orientation = Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
      return {position, orientation};

    }
  catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      // Mark this pose as invalid, will not write the error down
      return {Eigen::Vector3d (NULL, NULL, NULL), Eigen::Quaterniond (NULL, NULL, NULL, NULL)};
    }
}

double FloorplanDriftEstimator::computeDriftError(const Eigen::Vector3d gt_position, const Eigen::Vector3d current_position) {

  Eigen::Vector3d relativePosition = gt_position - current_position;

  // Extract euclidean distance between the positions and the angle between the orientations
  double translationError = relativePosition.norm();

  return translationError;
}

}  // namespace cost_computer
}  // namespace active_3d_planning
