#include "servo_py/servo.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace servo_py {
namespace {
void check_q(const Vector& q, int n) {
  if (q.size() != n || !q.allFinite())
    throw std::invalid_argument("q must be a finite vector with model.dof() entries");
}
Pose motion(const Joint& joint, double q) {
  Pose transform = Pose::Identity();
  if (joint.type == JointType::PRISMATIC)
    transform.block<3, 1>(0, 3) = joint.axis * q;
  else
    transform.block<3, 3>(0, 0) = Eigen::AngleAxisd(q, joint.axis).toRotationMatrix();
  return transform;
}
}  // namespace

bool valid_pose(const Pose& pose) {
  if (!pose.allFinite()) return false;
  const Eigen::Matrix3d r = pose.block<3, 3>(0, 0);
  return (pose.row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-8 &&
         (r.transpose() * r - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
         std::abs(r.determinant() - 1.0) < 1e-6;
}

Eigen::Vector3d rotation_log(const Eigen::Matrix3d& rotation) {
  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  // q and -q represent the same rotation; select the shortest rotation.
  if (quaternion.w() < 0) quaternion.coeffs() *= -1;
  const double norm = quaternion.vec().norm();
  if (norm < 1e-12) return 2.0 * quaternion.vec();
  return quaternion.vec() * (2.0 * std::atan2(norm, quaternion.w()) / norm);
}

void Limits::validate(int n) const {
  if (n <= 0 || lower.size() != n || upper.size() != n || velocity.size() != n ||
      acceleration.size() != n || margin.size() != n)
    throw std::invalid_argument("all joint limit arrays must have model.dof() entries");
  for (int i = 0; i < n; ++i) {
    if (std::isnan(lower[i]) || std::isnan(upper[i]) || !(lower[i] < upper[i]) ||
        !std::isfinite(velocity[i]) || velocity[i] <= 0 ||
        !std::isfinite(acceleration[i]) || acceleration[i] <= 0 ||
        !std::isfinite(margin[i]) || margin[i] < 0 ||
        2 * margin[i] >= upper[i] - lower[i])
      throw std::invalid_argument("invalid position, velocity, acceleration or margin limits");
  }
}

Vector Kinematics::integrate(const Vector& q, const Vector& delta) const {
  check_q(q, dof());
  check_q(delta, dof());
  return q + delta;
}
Vector Kinematics::difference(const Vector& q1, const Vector& q0) const {
  check_q(q1, dof());
  check_q(q0, dof());
  return q1 - q0;
}

SerialChain::SerialChain(std::vector<Joint> joints, Limits limits, std::string base, std::string tip)
    : joints_(std::move(joints)), limits_(std::move(limits)), base_(std::move(base)), tip_(std::move(tip)) {
  if (base_.empty() || tip_.empty() || base_ == tip_)
    throw std::invalid_argument("base and tip must be distinct, nonempty frame names");
  std::set<std::string> names;
  for (auto& joint : joints_) {
    if (joint.name.empty() || !names.insert(joint.name).second || !valid_pose(joint.origin))
      throw std::invalid_argument("joint names must be unique and origins must be rigid transforms");
    if (joint.type == JointType::FIXED) continue;
    if (joint.type != JointType::REVOLUTE && joint.type != JointType::CONTINUOUS &&
        joint.type != JointType::PRISMATIC)
      throw std::invalid_argument("unsupported joint type");
    if (!joint.axis.allFinite() || joint.axis.norm() < 1e-12)
      throw std::invalid_argument("joint axis must be finite and nonzero");
    joint.axis.normalize();
    names_.push_back(joint.name);
    continuous_.push_back(joint.type == JointType::CONTINUOUS);
  }
  limits_.validate(dof());
  for (int i = 0; i < dof(); ++i)
    if (continuous_[i] && (std::isfinite(limits_.lower[i]) || std::isfinite(limits_.upper[i])))
      throw std::invalid_argument("continuous joints require unbounded position limits");
}

int SerialChain::dof() const { return static_cast<int>(names_.size()); }
std::vector<std::string> SerialChain::joint_names() const { return names_; }
std::string SerialChain::base_frame() const { return base_; }
std::string SerialChain::tip_frame() const { return tip_; }
const Limits& SerialChain::limits() const { return limits_; }

Pose SerialChain::fk(const Vector& q) const {
  check_q(q, dof());
  Pose transform = Pose::Identity();
  int idx = 0;
  for (const auto& joint : joints_) {
    transform = (transform * joint.origin).eval();
    if (joint.type != JointType::FIXED)
      transform = (transform * motion(joint, q[idx++])).eval();
  }
  return transform;
}

Matrix SerialChain::jacobian(const Vector& q) const {
  check_q(q, dof());
  Matrix origins(3, dof()), axes(3, dof());
  std::vector<bool> prismatic;
  Pose transform = Pose::Identity();
  int idx = 0;
  for (const auto& joint : joints_) {
    transform = (transform * joint.origin).eval();
    if (joint.type == JointType::FIXED) continue;
    origins.col(idx) = transform.block<3, 1>(0, 3);
    axes.col(idx) = transform.block<3, 3>(0, 0) * joint.axis;
    prismatic.push_back(joint.type == JointType::PRISMATIC);
    transform = (transform * motion(joint, q[idx])).eval();
    ++idx;
  }
  const Eigen::Vector3d tip_position = transform.block<3, 1>(0, 3);
  Matrix jacobian = Matrix::Zero(6, dof());
  for (int i = 0; i < dof(); ++i) {
    if (prismatic[i]) {
      jacobian.block<3, 1>(0, i) = axes.col(i);
    } else {
      const Eigen::Vector3d axis = axes.col(i);
      jacobian.block<3, 1>(0, i) = axis.cross(tip_position - origins.col(i));
      jacobian.block<3, 1>(3, i) = axis;
    }
  }
  return jacobian;
}

Vector SerialChain::difference(const Vector& q1, const Vector& q0) const {
  Vector difference = Kinematics::difference(q1, q0);
  constexpr double two_pi = 6.28318530717958647692;
  for (int i = 0; i < dof(); ++i)
    if (continuous_[i]) difference[i] = std::remainder(difference[i], two_pi);
  return difference;
}
}  // namespace servo_py
