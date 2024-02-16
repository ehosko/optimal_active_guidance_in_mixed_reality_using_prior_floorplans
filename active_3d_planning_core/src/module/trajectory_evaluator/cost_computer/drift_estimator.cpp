#include "active_3d_planning_core/module/trajectory_evaluator/cost_computer/drift_estimator.h"

namespace active_3d_planning {
namespace cost_computer {

// SegmentTime
ModuleFactoryRegistry::Registration<DriftEstimator> DriftEstimator::registration(
    "DriftEstimator");

DriftEstimator::DriftEstimator(PlannerI& planner) : CostComputer(planner), tf_listener_(tf_buffer_) {}

void DriftEstimator::setupFromParamMap(Module::ParamMap* param_map) {
  // Needed in any case, want to accumulate from parents
  setParam<bool>(param_map, "accumulate", &p_accumulate_, true);
  // TODO: (ehosko) Adapt weight - was 1 before
  setParam<double>(param_map, "drift_weight", &drift_weight_, 0.5);
  setParam<double>(param_map, "orientation_error_weight", &orientation_error_weight_, 10);
  setParam<std::string>(param_map, "world_frame", &source_frame_, "world");
  setParam<std::string>(param_map, "ground_truth_frame", &gt_frame_, "firefly/base_link");
  setParam<std::string>(param_map, "floorplan_frame", &floorplan_frame_, "floorplan");
  setParam<std::string>(param_map, "drifty_frame", &drifty_frame_, "rovioli/imu");
  setParam<std::string>(param_map, "optimal_trajectory_frame", &opt_traj_frame, "optimal_trajectory");
  setParam<double>(param_map, "drift_estimation_radius", &drift_estimation_radius_, 0.5);
  setParam<double>(param_map, "drift_discount", &drift_discount_, 0.9);
  setParam<bool>(param_map,"use_opt_traj",&use_opt_traj_,false);
  setParam<bool>(param_map,"use_floorplan",&use_floorplan_asGT,false);
}

// TODO: (michbaum) This is the current cost computation used
bool DriftEstimator::computeCost(TrajectorySegment* traj_in) {
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
    //std::cout << "Segment Time cost: " << segment_time_cost << std::endl;
    // TODO: (but do this every time??) - adapt weight
    if(use_opt_traj_){
      // double opt_traj_error = 0.0;
      // for (int i = 0; i < traj_in->trajectory.size(); ++i) {
      //   opt_traj_error += computeOptTrajError(traj_in->trajectory[i].position_W,traj_in->trajectory[i].orientation_W_B);;
      //   //last_point = traj_in->trajectory[i].position_W;
      // }
      //traj_in->cost += (1 - drift_weight_) * opt_traj_error;
      //opt_traj_error = std::pow(opt_traj_error,2);
      double opt_traj_error = computeOptTrajError(current_position_back, current_orientation_back);
      //std::cout << "Optimal trajectory error: " << opt_traj_error << std::endl;
      traj_in->cost = opt_traj_error;
    }
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

  double drift_cost = 0;

  if(use_floorplan_asGT)
  {
    drift_cost = computeDriftErrorFloorplan(current_position_back, current_orientation_back);
  }
  else
  {
    drift_cost = computeDriftError(drifty_position, drifty_orientation);
  }

  // traj_in->cost = (1 - drift_weight_) * segment_time_cost + drift_weight_ * drift_cost;
  traj_in->cost = (1 - drift_weight_) * segment_time_cost + drift_weight_ * drift_cost;


  if(use_opt_traj_){
    // double opt_traj_error = 0.0;
    // for (int i = 0; i < traj_in->trajectory.size(); ++i) {
    //   opt_traj_error += computeOptTrajError(traj_in->trajectory[i].position_W,traj_in->trajectory[i].orientation_W_B);;
    //   //last_point = traj_in->trajectory[i].position_W;
    // }
    //opt_traj_error = std::pow(opt_traj_error,2);
    //traj_in->cost += (1 - drift_weight_) * opt_traj_error;
    double opt_traj_error = computeOptTrajError(current_position_back, current_orientation_back);
    //std::cout << "Optimal trajectory error: " << opt_traj_error << std::endl;
    // traj_in->cost = (1- drift_weight_) * opt_traj_error + drift_weight_ * drift_cost;
    traj_in->cost = opt_traj_error + drift_weight_ * drift_cost;
  }
      
  if (p_accumulate_ && traj_in->parent) {
    // TODO: Check that this really happens - it should!
    //traj_in->cost += traj_in->parent->cost;
    traj_in->cost += traj_in->parent->cost * drift_discount_;
  }
  return true;
}

Eigen::Vector3d transformToEigenPosition(const geometry_msgs::TransformStamped& transformStamped)
{
    Eigen::Vector3d position;
    position.x() = transformStamped.transform.translation.x;
    position.y() = transformStamped.transform.translation.y;
    position.z() = transformStamped.transform.translation.z;
    return position;
}

Eigen::Quaterniond transformToEigenQuaternion(const geometry_msgs::TransformStamped& transformStamped)
{
    Eigen::Quaterniond quaternion(transformStamped.transform.rotation.w, 
                                  transformStamped.transform.rotation.x, 
                                  transformStamped.transform.rotation.y, 
                                  transformStamped.transform.rotation.z);
    return quaternion;
}

std::tuple<Eigen::Vector3d, Eigen::Quaterniond> DriftEstimator::getCurrentPose(const std::string source_frame, const std::string target_frame){
  // Extract the transform from /world to the target frame at this point in time
  try
    {
      // Lookup the transform at the time of the pose message
      geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(target_frame, source_frame, ::ros::Time(0), ::ros::Duration(1.0));

      // Translate transform into Pose
      Eigen::Vector3d position = transformToEigenPosition(transform);
      Eigen::Quaterniond orientation = transformToEigenQuaternion(transform);
      return {position, orientation};

    }
  catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      // Mark this pose as invalid, will not write the error down
      return {Eigen::Vector3d (NULL, NULL, NULL), Eigen::Quaterniond (NULL, NULL, NULL, NULL)};
    }
}

double DriftEstimator::computeDriftError(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation) {

  // Extract the current ground truth pose
  std::tuple<Eigen::Vector3d, Eigen::Quaterniond> gt_pose = getCurrentPose(source_frame_, gt_frame_);
  Eigen::Vector3d gt_position = std::get<0>(gt_pose);
  Eigen::Quaterniond gt_orientation = std::get<1>(gt_pose);

  // Convert absolute poses to Eigen::Isometry3d
  Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
  pose1.translation() = current_position;
  pose1.linear() = current_orientation.toRotationMatrix();

  Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
  pose2.translation() = gt_position;
  pose2.linear() = gt_orientation.toRotationMatrix();

  // Calculate the relative pose
  Eigen::Isometry3d relativePose = pose1.inverse() * pose2;

  // Extract relative position and orientation
  Eigen::Vector3d relativePosition = relativePose.translation();
  Eigen::Quaterniond relativeOrientation(relativePose.linear());

  // Extract euclidean distance between the positions and the angle between the orientations
  double translationError = relativePosition.norm();
  Eigen::Vector3d euler = relativeOrientation.toRotationMatrix().eulerAngles(0, 1, 2);
  double angleError = euler.norm(); // Angle is the magnitude of the Euler angles vector

  return translationError + orientation_error_weight_ * angleError;
}

double DriftEstimator::computeDriftErrorFloorplan(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation)
{
    std::tuple<Eigen::Vector3d, Eigen::Quaterniond> floorplan_pose = getCurrentPose(floorplan_frame_, source_frame_);

    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();
    pose2.translation() = std::get<0>(floorplan_pose);
    pose2.linear() = std::get<1>(floorplan_pose).toRotationMatrix();

    Eigen::Vector3d projPos;
    projectOnFloor(current_position, current_orientation, projPos);

    // TODO: (ehosko) Check if this is the correct way to project the orientation (probably not, but it's a start)
    Eigen::Quaterniond projOrientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0,0,1), current_orientation * Eigen::Vector3d(0,0,1));

    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    pose1.translation() = projPos;
    pose1.linear() = projOrientation.toRotationMatrix();

    std::cout<<"Floorplan Pose: " << pose2.translation().x() << " " <<   pose2.translation().y() ; 

    //double translationError = (projPos - opt_traj_position).norm();

    // Calculate the relative pose
    Eigen::Isometry3d relativePose = pose1.inverse() * pose2;

    // Extract relative position and orientation
    Eigen::Vector3d relativePosition = relativePose.translation();
    Eigen::Quaterniond relativeOrientation(relativePose.linear());

    // Extract euclidean distance between the positions and the angle between the orientations
    double translationError = relativePosition.norm();
    Eigen::Vector3d euler = relativeOrientation.toRotationMatrix().eulerAngles(0, 1, 2);
    double angleError = euler.norm(); // Angle is the magnitude of the Euler angles vector

    // std::cout << "Floorplan Translation Drift Error: " << translationError << std::endl;
    // std::cout << "Floorplan Orientation Drift Error: " << angleError << std::endl;

    //return (projPos - std::get<0>(floorplan_pose)).norm();
    return translationError + angleError;
}

double DriftEstimator::computeOptTrajError(const Eigen::Vector3d current_position, const Eigen::Quaterniond current_orientation)
{
  std::tuple<Eigen::Vector3d, Eigen::Quaterniond> opt_traj_pose = getCurrentPose(opt_traj_frame, source_frame_);

  Eigen::Vector3d opt_traj_position = std::get<0>(opt_traj_pose);
  Eigen::Vector3d projPos;
  projectOnFloor(current_position, current_orientation, projPos);

  // std::cout << "Current Pose: " << current_position.x() << " " << current_position.y() << std::endl;
  // std::cout << "Projected Pose: " << projPos.x() << " " << projPos.y() << std::endl;
  // std::cout<<"Opt Traj Pose: " << opt_traj_position.x() << " " <<  opt_traj_position.y(); 

  double translationError = (projPos - opt_traj_position).norm();

  // std::cout << "Optimal trajectory error: " << translationError << std::endl;
  return translationError;
}

void DriftEstimator::projectOnFloor(Eigen::Vector3d pos, Eigen::Quaterniond q, Eigen::Vector3d& projVec)
{
  Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
  Eigen::Vector3d normalVector(0.0, 0.0, 1.0); // Assuming floor is horizontal
  normalVector = rotationMatrix * normalVector;

  // Project the point onto the floor
  double distance = -normalVector.dot(pos);
  projVec = pos - distance * normalVector;
  projVec.z() = 0;

  // Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
  // Eigen::Vector3d projectedVector = rotationMatrix.transpose() * pos;

  //projectedVector.z() = 0;
}

}  // namespace cost_computer
}  // namespace active_3d_planning
