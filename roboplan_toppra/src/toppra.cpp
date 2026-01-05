#include <chrono>
#include <stdexcept>

#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

PathParameterizerTOPPRA::PathParameterizerTOPPRA(const std::shared_ptr<Scene> scene,
                                                 const std::string& group_name)
    : scene_{scene}, group_name_{group_name} {
  // Extract the joint group information.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name_);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize TOPP-RA path parameterizer: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Extract joint velocity + acceleration limits from scene.
  const auto maybe_collapsed_pos = collapseContinuousJointPositions(
      *scene_, group_name_, Eigen::VectorXd::Zero(joint_group_info_.q_indices.size()));
  if (!maybe_collapsed_pos) {
    throw std::runtime_error("Failed to instantiate TOPP-RA: " + maybe_collapsed_pos.error());
  }
  const auto num_dofs = maybe_collapsed_pos->size();
  vel_lower_limits_ = Eigen::VectorXd::Zero(num_dofs);
  vel_upper_limits_ = Eigen::VectorXd::Zero(num_dofs);
  acc_lower_limits_ = Eigen::VectorXd::Zero(num_dofs);
  acc_upper_limits_ = Eigen::VectorXd::Zero(num_dofs);

  size_t q_idx = 0;
  for (size_t j_idx = 0; j_idx < joint_group_info_.joint_names.size(); ++j_idx) {
    const auto& joint_name = joint_group_info_.joint_names.at(j_idx);
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Failed to instantiate TOPP-RA: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();

    switch (joint_info.type) {
    case JointType::FLOATING:
    case JointType::PLANAR:
      throw std::runtime_error("Multi-DOF joints not yet supported by TOPP-RA.");
    case JointType::CONTINUOUS:
      continuous_joint_indices_.push_back(j_idx);
      [[fallthrough]];
    default:  // Prismatic, revolute, or continuous, which are single-DOF in tangent space.
      if (joint_info.limits.max_velocity.size() == 0) {
        throw std::runtime_error("Velocity limit must be defined for joint '" + joint_name + "'.");
      }
      if (joint_info.limits.max_acceleration.size() == 0) {
        throw std::runtime_error("Acceleration limit must be defined for joint '" + joint_name +
                                 "'.");
      }
      const auto& max_vel = joint_info.limits.max_velocity[0];
      vel_lower_limits_(q_idx) = -max_vel;
      vel_upper_limits_(q_idx) = max_vel;
      const auto& max_acc = joint_info.limits.max_acceleration[0];
      acc_lower_limits_(q_idx) = -max_acc;
      acc_upper_limits_(q_idx) = max_acc;
      ++q_idx;
    }
  }
}

tl::expected<JointTrajectory, std::string>
PathParameterizerTOPPRA::generate(const JointPath& path, const double dt,
                                  const double velocity_scale, const double acceleration_scale) {
  const auto num_pts = path.positions.size();
  if (num_pts < 2) {
    return tl::make_unexpected("Path must have at least 2 points.");
  }
  if (dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if ((velocity_scale <= 0.0) || (velocity_scale > 1.0)) {
    return tl::make_unexpected("Velocity scale must be greater than 0.0 and less than 1.0.");
  }
  if ((acceleration_scale <= 0.0) || (acceleration_scale > 1.0)) {
    return tl::make_unexpected("Acceleration scale must be greater than 0.0 and less than 1.0.");
  }
  const auto& joint_names = joint_group_info_.joint_names;
  if ((joint_names.size() != path.joint_names.size()) ||
      !std::equal(joint_names.begin(), joint_names.end(), path.joint_names.begin())) {
    return tl::make_unexpected("Path joint names do not match the scene joint names.");
  }

  // Create scaled velocity and acceleration constraints.
  toppra::LinearConstraintPtr vel_constraint, acc_constraint;
  vel_constraint = std::make_shared<toppra::constraint::LinearJointVelocity>(
      vel_lower_limits_ * velocity_scale, vel_upper_limits_ * velocity_scale);
  acc_constraint = std::make_shared<toppra::constraint::LinearJointAcceleration>(
      acc_lower_limits_ * acceleration_scale, acc_upper_limits_ * acceleration_scale);
  acc_constraint->discretizationType(toppra::DiscretizationType::Interpolation);
  toppra::LinearConstraintPtrs constraints = {vel_constraint, acc_constraint};

  // Create initial cubic spline with path points.
  toppra::Vectors path_pos_vecs;
  path_pos_vecs.reserve(num_pts);
  for (size_t idx = 0; idx < path.positions.size(); ++idx) {
    const auto& pos = path.positions.at(idx);
    auto maybe_collapsed_pos = collapseContinuousJointPositions(*scene_, group_name_, pos);
    if (!maybe_collapsed_pos) {
      return tl::make_unexpected("Failed to compute path parameterization: " +
                                 maybe_collapsed_pos.error());
    }
    auto curr_collapsed = maybe_collapsed_pos.value();

    // For continuous joints we have to ensure that we take "the short way around" in the spline.
    // If the distance to the preview point is greater than PI, then we either add or subtract
    // 2*PI to this point to ensure that we don't travel further than we need to.
    if (idx > 0) {
      const auto& prev_collapsed = path_pos_vecs.at(idx - 1);
      for (auto j_idx : continuous_joint_indices_) {
        const auto diff = curr_collapsed(j_idx) - prev_collapsed(j_idx);
        if (diff > M_PI) {
          curr_collapsed(j_idx) -= 2.0 * M_PI;
        } else if (diff < -M_PI) {
          curr_collapsed(j_idx) += 2.0 * M_PI;
        }
      }
    }
    path_pos_vecs.push_back(curr_collapsed);
  }

  // Iterative Spline Refinement for Collision Avoidance
  // We loop to generate the spline, check for collisions, and insert waypoints if needed.
  std::shared_ptr<toppra::PiecewisePolyPath> geom_path;
  const int kMaxIterations = 10;
  bool found_valid_path = false;

  // Collision checking helper.
  auto check_collision = [&](const Eigen::VectorXd& q_collapsed)
      -> tl::expected<bool, std::string> {
    auto maybe_expanded = expandContinuousJointPositions(*scene_, group_name_, q_collapsed);
    if (!maybe_expanded) {
      return tl::make_unexpected("Failed to expand joint positions: " +
                                 maybe_expanded.error());
    }
    const Eigen::VectorXd full_q =
        scene_->toFullJointPositions(group_name_, maybe_expanded.value());
    return scene_->hasCollisions(full_q);
  };

  for (int iter = 0; iter < kMaxIterations; ++iter) {
    // 1. Re-calculate steps based on current path points (chordal parameterization)
    std::vector<double> steps;
    steps.reserve(path_pos_vecs.size());
    double s = 0.0;
    steps.push_back(s);
    for (size_t i = 1; i < path_pos_vecs.size(); ++i) {
      double dist = (path_pos_vecs[i] - path_pos_vecs[i - 1]).norm();
      s += std::max(dist, 1e-4);
      steps.push_back(s);
    }

    // 2. Build Cubic Spline with clamped endpoint derivatives (smooth velocity transition)
    const Eigen::VectorXd start_deriv =
        (path_pos_vecs[1] - path_pos_vecs[0]) / (steps[1] - steps[0]);
    const Eigen::VectorXd end_deriv =
        (path_pos_vecs[path_pos_vecs.size() - 1] - path_pos_vecs[path_pos_vecs.size() - 2]) /
        (steps[steps.size() - 1] - steps[steps.size() - 2]);
    std::array<toppra::BoundaryCond, 2> bc_type = {
        toppra::BoundaryCond(1, start_deriv), toppra::BoundaryCond(1, end_deriv)};
    Eigen::Map<Eigen::VectorXd> spline_times_vec(steps.data(), steps.size());
    const auto spline =
        toppra::PiecewisePolyPath::CubicSpline(path_pos_vecs, spline_times_vec, bc_type);
    geom_path = std::make_shared<toppra::PiecewisePolyPath>(spline);

    // 3. Check for collisions
    bool collision_detected = false;
    size_t segment_to_split = 0;
    double split_alpha = 0.5;

    // Always verify that all waypoints are collision-free (including inserted ones).
    for (size_t i = 0; i < path_pos_vecs.size(); ++i) {
      const auto maybe_collision = check_collision(path_pos_vecs[i]);
      if (!maybe_collision) {
        return tl::make_unexpected(maybe_collision.error());
      }
      if (maybe_collision.value()) {
        return tl::make_unexpected("Path waypoint is in collision at index " +
                                   std::to_string(i) + ".");
      }
    }
    
    // Adaptive check resolution in joint space (approx 0.05 rad / 5 cm).
    const double max_joint_step = 0.05;
    
    for (size_t i = 0; i < steps.size() - 1; ++i) {
      const double t_start = steps[i];
      const double t_end = steps[i+1];
      
      const Eigen::VectorXd delta = path_pos_vecs[i + 1] - path_pos_vecs[i];
      const double max_delta = delta.cwiseAbs().maxCoeff();
      const int num_checks = std::max(1, static_cast<int>(std::ceil(max_delta / max_joint_step)));

      // Check intermediate points (exclude exact endpoints).
      for (int k = 1; k <= num_checks; ++k) {
        const double alpha = static_cast<double>(k) / (num_checks + 1);
        const double t = t_start + alpha * (t_end - t_start);
        const Eigen::VectorXd q_collapsed = geom_path->eval_single(t, 0);
        const auto maybe_collision = check_collision(q_collapsed);
        if (!maybe_collision) {
          return tl::make_unexpected(maybe_collision.error());
        }
        if (maybe_collision.value()) {
          collision_detected = true;
          segment_to_split = i;
          split_alpha = alpha;
          break;
        }
      }
      if (collision_detected) break;
    }

    if (!collision_detected) {
      found_valid_path = true;
      break;
    }

    // 4. Resolve Collision: Insert a point on the chord at the detected collision fraction.
    // This biases the path toward the straight-line segment which is more likely to be collision-free.
    if (iter < kMaxIterations - 1) {
        const Eigen::VectorXd chord_point =
            (1.0 - split_alpha) * path_pos_vecs[segment_to_split] +
            split_alpha * path_pos_vecs[segment_to_split + 1];
        const auto maybe_collision = check_collision(chord_point);
        if (!maybe_collision) {
          return tl::make_unexpected(maybe_collision.error());
        }
        if (maybe_collision.value()) {
          return tl::make_unexpected(
              "Refinement failed: chord point is in collision for segment " +
              std::to_string(segment_to_split) + ".");
        }
        path_pos_vecs.insert(path_pos_vecs.begin() + segment_to_split + 1, chord_point);
        // Continue to next iteration to rebuild spline
    }
  }
  
  if (!found_valid_path) {
      return tl::make_unexpected("Failed to find collision-free spline parameterization after " + 
                                 std::to_string(kMaxIterations) + " iterations.");
  }

  // Solve TOPP-RA problem.
  toppra::PathParametrizationAlgorithmPtr algo =
      std::make_shared<toppra::algorithm::TOPPRA>(constraints, geom_path);
  algo->setN(500);  // increase the number of grid points to prevent sharp acceleration changes
  const auto rc = algo->computePathParametrization();
  if (rc != toppra::ReturnCode::OK) {
    return tl::make_unexpected("TOPPRA failed with return code " +
                               std::to_string(static_cast<int>(rc)));
  }

  // Evaluate the parameterized path at the specified times.
  const auto param_data = algo->getParameterizationData();
  const auto const_acc = std::make_shared<toppra::parametrizer::ConstAccel>(
      geom_path, param_data.gridpoints, param_data.parametrization);

  JointTrajectory traj;
  traj.joint_names = path.joint_names;

  const auto t_final = const_acc->pathInterval()[1];
  const auto num_traj_pts = static_cast<size_t>(std::ceil(t_final / dt)) + 1;
  traj.times.reserve(num_traj_pts);
  traj.positions.reserve(num_traj_pts);
  traj.velocities.reserve(num_traj_pts);
  traj.accelerations.reserve(num_traj_pts);
  for (size_t i = 0; i < num_traj_pts; ++i) {
    const auto t = std::min(static_cast<double>(i) * dt, t_final);
    traj.times.push_back(t);
  }
  Eigen::Map<Eigen::VectorXd> times_vec(traj.times.data(), traj.times.size());
  for (const auto& pos : const_acc->eval(times_vec, 0)) {
    const auto maybe_expanded_pos = expandContinuousJointPositions(*scene_, group_name_, pos);
    if (!maybe_expanded_pos) {
      return tl::make_unexpected("Failed to compute path parameterization: " +
                                 maybe_expanded_pos.error());
    }
    traj.positions.push_back(maybe_expanded_pos.value());
  }
  for (const auto& vel : const_acc->eval(times_vec, 1)) {
    traj.velocities.push_back(vel);
  }
  for (const auto& acc : const_acc->eval(times_vec, 2)) {
    traj.accelerations.push_back(acc);
  }

  return traj;
}

}  // namespace roboplan
