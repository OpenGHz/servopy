#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace servo_py {
using Vector = Eigen::VectorXd;
using Matrix = Eigen::MatrixXd;
using Pose = Eigen::Matrix4d;
using Vector6 = Eigen::Matrix<double, 6, 1>;

enum class JointType { FIXED, REVOLUTE, CONTINUOUS, PRISMATIC };
enum class CommandType { JOINT_JOG, TWIST, POSE, STOP, JOINT_POSITION };
enum class Frame { BASE, TOOL };
enum class Action { TRACK, BRAKE, HOLD, REJECT };
enum Flag : std::uint64_t {
  NONE = 0,
  INVALID_COMMAND = 1ULL << 0,
  INVALID_STATE = 1ULL << 1,
  INVALID_TIMING = 1ULL << 2,
  STALE_COMMAND = 1ULL << 3,
  STALE_STATE = 1ULL << 4,
  FUTURE_TIMESTAMP = 1ULL << 5,
  TRACKING_ERROR = 1ULL << 6,
  VELOCITY_LIMIT = 1ULL << 7,
  ACCELERATION_LIMIT = 1ULL << 8,
  POSITION_LIMIT = 1ULL << 9,
  SINGULARITY_DECELERATION = 1ULL << 10,
  SINGULARITY_HALT = 1ULL << 11,
  LEAVING_SINGULARITY = 1ULL << 12,
  COLLISION_DISABLED = 1ULL << 13,
  COLLISION_MISSING = 1ULL << 14,
  COLLISION_STALE = 1ULL << 15,
  COLLISION_DECELERATION = 1ULL << 16,
  COLLISION_HALT = 1ULL << 17,
  INFEASIBLE = 1ULL << 18,
  FAULT_LATCHED = 1ULL << 19,
  GOAL_REACHED = 1ULL << 20,
  MODE_SWITCH = 1ULL << 21,
  MODEL_ERROR = 1ULL << 22,
};

struct Limits {
  Vector lower, upper, velocity, acceleration, margin;
  void validate(int n) const;
};

struct Joint {
  std::string name;
  JointType type = JointType::REVOLUTE;
  Pose origin = Pose::Identity();  // parent -> joint at zero configuration
  Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
};

class Kinematics {
 public:
  virtual ~Kinematics() = default;
  virtual int dof() const = 0;
  virtual std::vector<std::string> joint_names() const = 0;
  virtual std::string base_frame() const = 0;
  virtual std::string tip_frame() const = 0;
  virtual Pose fk(const Vector& q) const = 0;
  // [TCP linear velocity; angular velocity], both expressed in base axes.
  virtual Matrix jacobian(const Vector& q) const = 0;
  virtual Vector integrate(const Vector& q, const Vector& delta) const;
  virtual Vector difference(const Vector& q1, const Vector& q0) const;
};

class SerialChain final : public Kinematics {
 public:
  SerialChain(std::vector<Joint> joints, Limits limits,
              std::string base = "base", std::string tip = "tool");
  int dof() const override;
  std::vector<std::string> joint_names() const override;
  std::string base_frame() const override;
  std::string tip_frame() const override;
  Pose fk(const Vector& q) const override;
  Matrix jacobian(const Vector& q) const override;
  Vector difference(const Vector& q1, const Vector& q0) const override;
  const Limits& limits() const;
 private:
  std::vector<Joint> joints_;
  std::vector<std::string> names_;
  std::vector<bool> continuous_;
  Limits limits_;
  std::string base_, tip_;
};

struct Config {
  double command_timeout = 0.1;
  double state_timeout = 0.1;
  double collision_timeout = 0.1;
  double max_dt = 0.05;
  double timing_tolerance = 0.5;  // fraction of preceding interval
  double max_tracking_error = 0.2;  // max absolute joint error, rad or m
  double position_gain = 2.0;
  double orientation_gain = 2.0;
  double joint_position_gain = 2.0;
  double joint_position_tolerance = 1e-4;  // rad or m, applied per joint
  double max_linear_speed = 0.2;
  double max_angular_speed = 0.5;
  double position_tolerance = 1e-4;
  double orientation_tolerance = 1e-3;
  double min_damping = 1e-4;
  double max_damping = 0.1;
  double damping_threshold = 0.1;
  double singularity_soft = 0.05;
  double singularity_hard = 0.001;
  double singularity_probe_step = 0.01;
  double singularity_escape_epsilon = 1e-6;
  bool collision_required = false;
  std::vector<int> task_axes{0, 1, 2, 3, 4, 5};
  Vector6 task_weights = Vector6::Ones();
  void validate(int n) const;
};

struct State {
  Vector q, dq;
  std::int64_t stamp_ns = 0;
};

struct Command {
  CommandType type = CommandType::STOP;
  Frame frame = Frame::BASE;
  Vector joint_velocity;
  Vector joint_position;
  Eigen::Vector3d linear = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular = Eigen::Vector3d::Zero();
  Pose pose = Pose::Identity();
  std::int64_t stamp_ns = 0;
  bool valid = true;
};

struct CollisionSample {
  double velocity_scale = 1.0;
  std::int64_t stamp_ns = 0;
  std::int64_t state_stamp_ns = 0;
};

struct Reference {
  Vector q, dq, ddq;
  std::int64_t stamp_ns = 0;
};

struct Diagnostics {
  double sigma_min = 0.0;
  double damping = 0.0;
  double singularity_scale = 1.0;
  double velocity_scale = 1.0;
  double collision_scale = 1.0;
  double tracking_error = 0.0;
  double position_error = 0.0;
  double orientation_error = 0.0;
  double joint_position_error = 0.0;  // target versus measured joints, max norm
};

struct Result {
  Action action = Action::REJECT;
  std::optional<Reference> reference;
  std::uint64_t flags = NONE;
  Diagnostics diagnostics;
  std::string message;
};

class ServoCore {
 public:
  ServoCore(std::shared_ptr<Kinematics> model, Limits limits, Config config = {});
  Result step(const State& state, const Command& command, double dt,
              std::int64_t now_ns,
              const std::optional<CollisionSample>& collision = std::nullopt);
  void reset(const State& state, std::int64_t now_ns);
 private:
  Result reject(Result result, std::uint64_t flags, const std::string& message);
  std::shared_ptr<Kinematics> model_;
  Limits limits_;
  Config config_;
  int n_;
  std::optional<Reference> reference_;
  std::optional<std::int64_t> last_now_;
  std::optional<CommandType> previous_type_;
  double previous_dt_ = 0;
  bool fault_latched_ = false;
  std::mutex mutex_;
};

bool valid_pose(const Pose& pose);
Eigen::Vector3d rotation_log(const Eigen::Matrix3d& rotation);
}  // namespace servo_py
