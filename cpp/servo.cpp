#include "servo_py/servo.hpp"
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace servo_py {
namespace {
constexpr double eps = 1e-10;

bool positive(double value) { return std::isfinite(value) && value > 0; }
double age(std::int64_t now, std::int64_t stamp) {
  // Subtract integer timestamps first to retain precision after long uptimes.
  return static_cast<double>(now - stamp) * 1e-9;
}
bool state_valid(const State& state, int n) {
  return state.q.size() == n && state.dq.size() == n &&
         state.q.allFinite() && state.dq.allFinite() && state.stamp_ns >= 0;
}
void cap_norm(Eigen::Vector3d& vector, double limit) {
  const double norm = vector.stableNorm();
  if (norm > limit) vector *= limit / norm;
}
Matrix select_rows(const Matrix& jacobian, const Config& config, int n) {
  if (jacobian.rows() != 6 || jacobian.cols() != n || !jacobian.allFinite())
    throw std::runtime_error("backend returned an invalid Jacobian");
  Matrix selected(config.task_axes.size(), n);
  for (std::size_t i = 0; i < config.task_axes.size(); ++i) {
    const int axis = config.task_axes[i];
    selected.row(i) = config.task_weights[axis] * jacobian.row(axis);
  }
  if (!selected.allFinite()) throw std::runtime_error("weighted Jacobian overflow");
  return selected;
}
double minimum_singular_value(const Matrix& matrix) {
  Eigen::JacobiSVD<Matrix> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  return svd.singularValues().tail(1)[0];
}

// Maximum positive excursion during a constant-acceleration interval, followed
// by deceleration at the same sampling period. The final fractional velocity
// is brought to zero over one full interval; using continuous stopping distance
// alone would require a mid-interval stop or an unwanted reversal.
// This function is monotone in v1, allowing a bounded scalar feasibility solve.
double stopping_excursion(double v0, double v1, double dt, double acceleration) {
  if (v1 >= 0) {
    const double decrement = acceleration * dt;
    const double remainder = std::fmod(v1, decrement);
    const double sampled_stop = v1 * v1 / (2 * acceleration) +
      remainder * (decrement - remainder) / (2 * acceleration);
    return std::max(0.0, 0.5 * dt * (v0 + v1) + sampled_stop);
  }
  if (v0 > 0) return 0.5 * dt * v0 * v0 / (v0 - v1);
  return 0.0;
}

bool feasible_velocity_interval(double q, double v0, int i, const Limits& limits,
                                double dt, double& lower, double& upper) {
  const double a = limits.acceleration[i];
  if (!(a * dt > 0) || !std::isfinite(a * dt)) return false;
  lower = std::max(-limits.velocity[i], v0 - a * dt);
  upper = std::min(limits.velocity[i], v0 + a * dt);
  if (lower > upper) return false;
  // Starting within a margin permits retreat, but never moving further outward.
  const double q_min = std::min(q, limits.lower[i] + limits.margin[i]);
  const double q_max = std::max(q, limits.upper[i] - limits.margin[i]);
  const double above = q_max - q;
  const double below = q - q_min;
  if (std::isfinite(above)) {
    if (stopping_excursion(v0, lower, dt, a) > above + eps) return false;
    if (stopping_excursion(v0, upper, dt, a) > above) {
      double lo = lower, hi = upper;
      for (int k = 0; k < 50; ++k) {
        const double mid = 0.5 * (lo + hi);
        if (stopping_excursion(v0, mid, dt, a) <= above) lo = mid;
        else hi = mid;
      }
      upper = lo;
    }
  }
  if (std::isfinite(below)) {
    if (stopping_excursion(-v0, -upper, dt, a) > below + eps) return false;
    if (stopping_excursion(-v0, -lower, dt, a) > below) {
      double lo = lower, hi = upper;
      for (int k = 0; k < 50; ++k) {
        const double mid = 0.5 * (lo + hi);
        if (stopping_excursion(-v0, -mid, dt, a) <= below) hi = mid;
        else lo = mid;
      }
      lower = hi;
    }
  }
  return lower <= upper;
}
}  // namespace

void Config::validate(int n) const {
  for (double value : {command_timeout, state_timeout, collision_timeout, max_dt,
                       max_tracking_error, position_gain, orientation_gain,
                       joint_position_gain, joint_position_tolerance,
                       max_linear_speed, max_angular_speed, position_tolerance,
                       orientation_tolerance, min_damping, max_damping,
                       damping_threshold, singularity_soft, singularity_hard,
                       singularity_probe_step, singularity_escape_epsilon})
    if (!positive(value)) throw std::invalid_argument("config scalars must be finite and positive");
  if (!std::isfinite(timing_tolerance) || timing_tolerance < 0 || timing_tolerance > 1 ||
      min_damping > max_damping || singularity_hard >= singularity_soft ||
      max_dt > 1.0 || task_axes.empty() || static_cast<int>(task_axes.size()) > n)
    throw std::invalid_argument("invalid timing, damping, singularity thresholds or task dimension");
  std::set<int> axes;
  for (int axis : task_axes)
    if (axis < 0 || axis > 5 || !axes.insert(axis).second)
      throw std::invalid_argument("task_axes must be unique values in [0, 5]");
  if (!task_weights.allFinite() || (task_weights.array() <= 0).any())
    throw std::invalid_argument("task weights must be finite and positive");
}

ServoCore::ServoCore(std::shared_ptr<Kinematics> model, Limits limits, Config config)
    : model_(std::move(model)), limits_(std::move(limits)), config_(std::move(config)) {
  if (!model_) throw std::invalid_argument("model must not be null");
  n_ = model_->dof();
  limits_.validate(n_);
  config_.validate(n_);
  const auto names = model_->joint_names();
  if (static_cast<int>(names.size()) != n_ || std::set<std::string>(names.begin(), names.end()).size() != names.size())
    throw std::invalid_argument("backend joint names must match its dimension and be unique");
}

void ServoCore::reset(const State& state, std::int64_t now_ns) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) throw std::runtime_error("ServoCore is already in use");
  if (!state_valid(state, n_) || now_ns < state.stamp_ns || now_ns < 0 ||
      age(now_ns, state.stamp_ns) > config_.state_timeout ||
      (state.q.array() < limits_.lower.array()).any() ||
      (state.q.array() > limits_.upper.array()).any() ||
      (state.dq.cwiseAbs().array() > limits_.velocity.array() + eps).any())
    throw std::invalid_argument("reset requires a fresh, finite state within physical limits");
  reference_ = Reference{state.q, state.dq, Vector::Zero(n_), now_ns};
  last_now_.reset();
  previous_type_.reset();
  fault_latched_ = false;
}

Result ServoCore::reject(Result result, std::uint64_t flags, const std::string& message) {
  result.action = Action::REJECT;
  result.reference.reset();
  result.flags |= flags;
  result.message = message;
  fault_latched_ = true;
  return result;
}

Result ServoCore::step(const State& state, const Command& command, double dt,
                       std::int64_t now_ns, const std::optional<CollisionSample>& collision) {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) throw std::runtime_error("ServoCore is already in use");
  Result result;
  if (!collision && !config_.collision_required) result.flags |= COLLISION_DISABLED;
  if (fault_latched_) return reject(result, FAULT_LATCHED, "fault is latched; reset with a fresh state");
  if (!positive(dt) || dt > config_.max_dt || dt < 1e-9 || now_ns < 0 ||
      now_ns > std::numeric_limits<std::int64_t>::max() - std::llround(dt * 1e9))
    return reject(result, INVALID_TIMING, "invalid dt or monotonic timestamp");
  if (last_now_) {
    const double elapsed = age(now_ns, *last_now_);
    if (now_ns <= *last_now_ || std::abs(elapsed - previous_dt_) > config_.timing_tolerance * previous_dt_ + 1e-9)
      return reject(result, INVALID_TIMING, "call time does not match the preceding reference interval");
  }
  if (!state_valid(state, n_)) return reject(result, INVALID_STATE, "invalid joint state");
  if (state.stamp_ns > now_ns) return reject(result, FUTURE_TIMESTAMP | INVALID_STATE, "joint state is from the future");
  if (age(now_ns, state.stamp_ns) > config_.state_timeout)
    return reject(result, STALE_STATE, "joint feedback expired");
  if ((state.q.array() < limits_.lower.array() - eps).any() ||
      (state.q.array() > limits_.upper.array() + eps).any())
    return reject(result, INVALID_STATE | POSITION_LIMIT, "measured state is outside physical joint limits");
  if ((state.dq.cwiseAbs().array() > limits_.velocity.array() + eps).any())
    return reject(result, INVALID_STATE | VELOCITY_LIMIT, "measured velocity is outside configured limits");
  if (!reference_) {
    reference_ = Reference{state.q, state.dq, Vector::Zero(n_), now_ns};
  }

  try {
    const Vector error = model_->difference(reference_->q, state.q);
    if (error.size() != n_ || !error.allFinite()) throw std::runtime_error("backend difference is invalid");
    result.diagnostics.tracking_error = error.cwiseAbs().maxCoeff();
    if (result.diagnostics.tracking_error > config_.max_tracking_error)
      return reject(result, TRACKING_ERROR, "measured joints no longer track the generated reference");

    if (!collision && config_.collision_required)
      return reject(result, COLLISION_MISSING, "collision monitoring is required");
    if (collision) {
      if (!std::isfinite(collision->velocity_scale) || collision->velocity_scale < 0 || collision->velocity_scale > 1 ||
          collision->stamp_ns < 0 || collision->state_stamp_ns < 0 ||
          collision->stamp_ns > now_ns || collision->state_stamp_ns > collision->stamp_ns)
        return reject(result, COLLISION_STALE, "invalid collision sample");
      if (age(now_ns, collision->stamp_ns) > config_.collision_timeout ||
          age(now_ns, collision->state_stamp_ns) > config_.collision_timeout)
        return reject(result, COLLISION_STALE, "collision result or its source state expired");
      result.diagnostics.collision_scale = collision->velocity_scale;
      if (collision->velocity_scale == 0)
        return reject(result, COLLISION_HALT, "collision halt requested; downstream stop is required");
      if (collision->velocity_scale < 1) result.flags |= COLLISION_DECELERATION;
    }

    Vector desired = Vector::Zero(n_);
    bool braking = command.type == CommandType::STOP;
    bool valid = command.valid && command.stamp_ns >= 0;
    if (command.stamp_ns > now_ns) { valid = false; result.flags |= FUTURE_TIMESTAMP; }
    if (!valid) { braking = true; result.flags |= INVALID_COMMAND; }
    else if (command.type != CommandType::STOP && age(now_ns, command.stamp_ns) > config_.command_timeout) {
      braking = true; result.flags |= STALE_COMMAND;
    }
    if (previous_type_ && *previous_type_ != command.type) result.flags |= MODE_SWITCH;

    if (!braking && command.type == CommandType::JOINT_JOG) {
      if (command.joint_velocity.size() != n_ || !command.joint_velocity.allFinite()) {
        braking = true; result.flags |= INVALID_COMMAND;
      } else desired = command.joint_velocity;
    } else if (!braking && command.type == CommandType::JOINT_POSITION) {
      const Vector& target = command.joint_position;
      if (target.size() != n_ || !target.allFinite() ||
          (target.array() < limits_.lower.array() + limits_.margin.array()).any() ||
          (target.array() > limits_.upper.array() - limits_.margin.array()).any()) {
        braking = true; result.flags |= INVALID_COMMAND;
      } else {
        const Vector reference_error = model_->difference(target, reference_->q);
        const Vector measured_error = model_->difference(target, state.q);
        if (reference_error.size() != n_ || measured_error.size() != n_ ||
            !reference_error.allFinite() || !measured_error.allFinite())
          throw std::runtime_error("backend returned an invalid joint position difference");
        result.diagnostics.joint_position_error = measured_error.cwiseAbs().maxCoeff();
        if (reference_error.cwiseAbs().maxCoeff() <= config_.joint_position_tolerance) {
          braking = true;
          if (result.diagnostics.joint_position_error <= config_.joint_position_tolerance)
            result.flags |= GOAL_REACHED;
        } else {
          desired = config_.joint_position_gain * reference_error;
          if (!desired.allFinite()) { braking = true; result.flags |= INVALID_COMMAND; }
        }
      }
    } else if (!braking && (command.type == CommandType::TWIST || command.type == CommandType::POSE)) {
      const Pose current = model_->fk(state.q);
      if (!valid_pose(current)) throw std::runtime_error("backend FK is not a rigid transform");
      Eigen::Vector3d linear = Eigen::Vector3d::Zero(), angular = Eigen::Vector3d::Zero();
      if (command.type == CommandType::TWIST) {
        valid = command.linear.allFinite() && command.angular.allFinite() &&
                (command.frame == Frame::BASE || command.frame == Frame::TOOL);
        if (valid) {
          linear = command.linear;
          angular = command.angular;
          if (command.frame == Frame::TOOL) {
            linear = (current.block<3, 3>(0, 0) * linear).eval();
            angular = (current.block<3, 3>(0, 0) * angular).eval();
          }
        }
      } else {
        valid = command.frame == Frame::BASE && valid_pose(command.pose);
        if (valid) {
          const Eigen::Vector3d dp = command.pose.block<3, 1>(0, 3) - current.block<3, 1>(0, 3);
          const Eigen::Vector3d dr = rotation_log(command.pose.block<3, 3>(0, 0) * current.block<3, 3>(0, 0).transpose());
          double dp2 = 0, dr2 = 0;
          for (int axis : config_.task_axes) {
            if (axis < 3) dp2 += dp[axis] * dp[axis];
            else dr2 += dr[axis - 3] * dr[axis - 3];
          }
          result.diagnostics.position_error = std::sqrt(dp2);
          result.diagnostics.orientation_error = std::sqrt(dr2);
          linear = config_.position_gain * dp;
          angular = config_.orientation_gain * dr;
          if (std::sqrt(dp2) <= config_.position_tolerance && std::sqrt(dr2) <= config_.orientation_tolerance) {
            braking = true; result.flags |= GOAL_REACHED;
          }
        }
      }
      if (!valid || !linear.allFinite() || !angular.allFinite()) {
        braking = true; result.flags |= INVALID_COMMAND;
      }
      if (!braking) {
        // Uncontrolled components must not consume the task speed budget.
        for (int axis = 0; axis < 6; ++axis)
          if (std::find(config_.task_axes.begin(), config_.task_axes.end(), axis) == config_.task_axes.end()) {
            if (axis < 3) linear[axis] = 0;
            else angular[axis - 3] = 0;
          }
        cap_norm(linear, config_.max_linear_speed);
        cap_norm(angular, config_.max_angular_speed);
        Vector6 twist; twist << linear, angular;
        Matrix j = select_rows(model_->jacobian(state.q), config_, n_);
        Vector task(config_.task_axes.size());
        for (std::size_t i = 0; i < config_.task_axes.size(); ++i)
          task[i] = config_.task_weights[config_.task_axes[i]] * twist[config_.task_axes[i]];
        Eigen::JacobiSVD<Matrix> svd(j, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto singular = svd.singularValues();
        const double sigma = singular.tail(1)[0];
        const double ratio = std::clamp(1.0 - sigma / config_.damping_threshold, 0.0, 1.0);
        const double damping = config_.min_damping + (config_.max_damping - config_.min_damping) * ratio * ratio;
        result.diagnostics.sigma_min = sigma;
        result.diagnostics.damping = damping;
        const Vector inverse = singular.array() / (singular.array().square() + damping * damping);
        desired = svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose() * task;
        if (task.norm() > eps && sigma < config_.singularity_soft) {
          bool escaping = false;
          if (desired.norm() > eps) {
            const Vector probe_q = model_->integrate(state.q, desired.normalized() * config_.singularity_probe_step);
            const double probe_sigma = minimum_singular_value(select_rows(model_->jacobian(probe_q), config_, n_));
            escaping = probe_sigma > sigma + config_.singularity_escape_epsilon;
          }
          if (escaping) result.flags |= LEAVING_SINGULARITY;
          else {
            const double scale = std::clamp((sigma - config_.singularity_hard) /
              (config_.singularity_soft - config_.singularity_hard), 0.0, 1.0);
            result.diagnostics.singularity_scale = scale;
            desired *= scale;
            result.flags |= scale == 0 ? SINGULARITY_HALT : SINGULARITY_DECELERATION;
            if (scale == 0) braking = true;
          }
        }
      }
    } else if (!braking) {
      braking = true; result.flags |= INVALID_COMMAND;
    }

    if (braking) desired.setZero();
    if (!desired.allFinite()) return reject(result, MODEL_ERROR, "nonfinite velocity from kinematic solve");
    double velocity_scale = 1.0;
    for (int i = 0; i < n_; ++i)
      if (std::abs(desired[i]) > limits_.velocity[i])
        velocity_scale = std::min(velocity_scale, limits_.velocity[i] / std::abs(desired[i]));
    if (velocity_scale < 1) result.flags |= VELOCITY_LIMIT;
    result.diagnostics.velocity_scale = velocity_scale;
    desired *= velocity_scale * result.diagnostics.collision_scale;
    if (desired.cwiseAbs().maxCoeff() <= eps) braking = true;

    Reference next;
    next.dq.resize(n_);
    for (int i = 0; i < n_; ++i) {
      const double v0 = reference_->dq[i];
      const double acceleration_limited = std::clamp(desired[i], v0 - limits_.acceleration[i] * dt,
                                                   v0 + limits_.acceleration[i] * dt);
      if (std::abs(acceleration_limited - desired[i]) > eps) result.flags |= ACCELERATION_LIMIT;
      double lo, hi;
      if (!feasible_velocity_interval(reference_->q[i], v0, i, limits_, dt, lo, hi))
        return reject(result, INFEASIBLE | POSITION_LIMIT, "no acceleration-bounded reference can respect joint limits");
      next.dq[i] = std::clamp(desired[i], lo, hi);
      if (std::abs(next.dq[i]) < 1e-12 && lo <= 0 && hi >= 0) next.dq[i] = 0.0;
      if (std::abs(next.dq[i] - acceleration_limited) > eps) result.flags |= POSITION_LIMIT;
    }
    const Vector delta = 0.5 * (reference_->dq + next.dq) * dt;
    next.q = model_->integrate(reference_->q, delta);
    next.ddq = (next.dq - reference_->dq) / dt;
    next.stamp_ns = now_ns + std::llround(dt * 1e9);
    if (next.q.size() != n_ || !next.q.allFinite() || !next.dq.allFinite() || !next.ddq.allFinite() ||
        (next.q.array() < limits_.lower.array() - eps).any() ||
        (next.q.array() > limits_.upper.array() + eps).any() ||
        (next.dq.cwiseAbs().array() > limits_.velocity.array() * (1 + 1e-10) + 1e-12).any() ||
        (next.ddq.cwiseAbs().array() > limits_.acceleration.array() * (1 + 1e-10) + 1e-12).any())
      return reject(result, INFEASIBLE, "final reference failed validation");
    const bool stationary = reference_->dq.cwiseAbs().maxCoeff() <= eps && next.dq.cwiseAbs().maxCoeff() <= eps;
    result.action = stationary ? Action::HOLD : (braking ? Action::BRAKE : Action::TRACK);
    result.reference = next;
    result.message = "reference generated";
    reference_ = std::move(next);
    last_now_ = now_ns;
    previous_dt_ = dt;
    previous_type_ = command.type;
    return result;
  } catch (const std::exception& error) {
    return reject(result, MODEL_ERROR, std::string("kinematic backend failed: ") + error.what());
  }
}
}  // namespace servo_py
