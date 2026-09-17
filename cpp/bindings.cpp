#include "servo_py/servo.hpp"
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace servo_py;

class PyKinematics : public Kinematics {
 public:
  using Kinematics::Kinematics;
  int dof() const override { PYBIND11_OVERRIDE_PURE(int, Kinematics, dof); }
  std::vector<std::string> joint_names() const override {
    PYBIND11_OVERRIDE_PURE(std::vector<std::string>, Kinematics, joint_names);
  }
  std::string base_frame() const override { PYBIND11_OVERRIDE_PURE(std::string, Kinematics, base_frame); }
  std::string tip_frame() const override { PYBIND11_OVERRIDE_PURE(std::string, Kinematics, tip_frame); }
  Pose fk(const Vector& q) const override { PYBIND11_OVERRIDE_PURE(Pose, Kinematics, fk, q); }
  Matrix jacobian(const Vector& q) const override { PYBIND11_OVERRIDE_PURE(Matrix, Kinematics, jacobian, q); }
  Vector integrate(const Vector& q, const Vector& delta) const override {
    PYBIND11_OVERRIDE(Vector, Kinematics, integrate, q, delta);
  }
  Vector difference(const Vector& q1, const Vector& q0) const override {
    PYBIND11_OVERRIDE(Vector, Kinematics, difference, q1, q0);
  }
};

PYBIND11_MODULE(_core, m) {
  m.doc() = "ROS-independent servo kernel (C++17/Eigen)";
  m.attr("__version__") = "0.1.0";
  py::enum_<JointType>(m, "JointType")
    .value("FIXED", JointType::FIXED).value("REVOLUTE", JointType::REVOLUTE)
    .value("CONTINUOUS", JointType::CONTINUOUS).value("PRISMATIC", JointType::PRISMATIC);
  py::enum_<CommandType>(m, "CommandType")
    .value("JOINT_JOG", CommandType::JOINT_JOG).value("TWIST", CommandType::TWIST)
    .value("POSE", CommandType::POSE).value("STOP", CommandType::STOP);
  py::enum_<Frame>(m, "Frame").value("BASE", Frame::BASE).value("TOOL", Frame::TOOL);
  py::enum_<Action>(m, "Action").value("TRACK", Action::TRACK).value("BRAKE", Action::BRAKE)
    .value("HOLD", Action::HOLD).value("REJECT", Action::REJECT);
#define FLAG(name) m.attr(#name) = py::int_(static_cast<std::uint64_t>(name));
  FLAG(NONE) FLAG(INVALID_COMMAND) FLAG(INVALID_STATE) FLAG(INVALID_TIMING)
  FLAG(STALE_COMMAND) FLAG(STALE_STATE) FLAG(FUTURE_TIMESTAMP) FLAG(TRACKING_ERROR)
  FLAG(VELOCITY_LIMIT) FLAG(ACCELERATION_LIMIT) FLAG(POSITION_LIMIT)
  FLAG(SINGULARITY_DECELERATION) FLAG(SINGULARITY_HALT) FLAG(LEAVING_SINGULARITY)
  FLAG(COLLISION_DISABLED) FLAG(COLLISION_MISSING) FLAG(COLLISION_STALE)
  FLAG(COLLISION_DECELERATION) FLAG(COLLISION_HALT) FLAG(INFEASIBLE) FLAG(FAULT_LATCHED)
  FLAG(GOAL_REACHED) FLAG(MODE_SWITCH) FLAG(MODEL_ERROR)
#undef FLAG
  py::class_<Limits>(m, "Limits").def(py::init<>())
    .def_readwrite("lower", &Limits::lower).def_readwrite("upper", &Limits::upper)
    .def_readwrite("velocity", &Limits::velocity).def_readwrite("acceleration", &Limits::acceleration)
    .def_readwrite("margin", &Limits::margin).def("validate", &Limits::validate);
  py::class_<Joint>(m, "Joint").def(py::init<>())
    .def_readwrite("name", &Joint::name).def_readwrite("type", &Joint::type)
    .def_readwrite("origin", &Joint::origin).def_readwrite("axis", &Joint::axis);
  py::class_<Kinematics, PyKinematics, std::shared_ptr<Kinematics>>(m, "Kinematics")
    .def(py::init<>()).def("dof", &Kinematics::dof).def("joint_names", &Kinematics::joint_names)
    .def("base_frame", &Kinematics::base_frame).def("tip_frame", &Kinematics::tip_frame)
    .def("fk", &Kinematics::fk).def("jacobian", &Kinematics::jacobian)
    .def("integrate", &Kinematics::integrate).def("difference", &Kinematics::difference);
  py::class_<SerialChain, Kinematics, std::shared_ptr<SerialChain>>(m, "SerialChainModel")
    .def(py::init<std::vector<Joint>, Limits, std::string, std::string>(),
      py::arg("joints"), py::arg("limits"), py::arg("base") = "base", py::arg("tip") = "tool")
    .def_property_readonly("limits", [](const SerialChain& model) { return Limits(model.limits()); });
  py::class_<Config> config(m, "Config");
  config.def(py::init<>());
#define FIELD(name) config.def_readwrite(#name, &Config::name);
  FIELD(command_timeout) FIELD(state_timeout) FIELD(collision_timeout) FIELD(max_dt)
  FIELD(timing_tolerance) FIELD(max_tracking_error) FIELD(position_gain) FIELD(orientation_gain)
  FIELD(max_linear_speed) FIELD(max_angular_speed) FIELD(position_tolerance) FIELD(orientation_tolerance)
  FIELD(min_damping) FIELD(max_damping) FIELD(damping_threshold)
  FIELD(singularity_soft) FIELD(singularity_hard) FIELD(singularity_probe_step) FIELD(singularity_escape_epsilon)
  FIELD(collision_required) FIELD(task_axes) FIELD(task_weights)
#undef FIELD
  py::class_<State>(m, "State")
    .def(py::init([](Vector q, Vector dq, std::int64_t stamp) { return State{std::move(q), std::move(dq), stamp}; }));
  py::class_<Command>(m, "Command").def(py::init<>())
    .def_readwrite("type", &Command::type).def_readwrite("frame", &Command::frame)
    .def_readwrite("joint_velocity", &Command::joint_velocity)
    .def_readwrite("linear", &Command::linear).def_readwrite("angular", &Command::angular)
    .def_readwrite("pose", &Command::pose).def_readwrite("stamp_ns", &Command::stamp_ns)
    .def_readwrite("valid", &Command::valid);
  py::class_<CollisionSample>(m, "CollisionSample")
    .def(py::init([](double scale, std::int64_t stamp, std::int64_t source) {
      return CollisionSample{scale, stamp, source}; }));
  py::class_<Reference>(m, "Reference")
    .def_property_readonly("q", [](const Reference& r) { return Vector(r.q); })
    .def_property_readonly("dq", [](const Reference& r) { return Vector(r.dq); })
    .def_property_readonly("ddq", [](const Reference& r) { return Vector(r.ddq); })
    .def_readonly("stamp_ns", &Reference::stamp_ns);
  py::class_<Diagnostics> diagnostics(m, "Diagnostics");
#define READ(name) diagnostics.def_readonly(#name, &Diagnostics::name);
  READ(sigma_min) READ(damping) READ(singularity_scale) READ(velocity_scale)
  READ(collision_scale) READ(tracking_error) READ(position_error) READ(orientation_error)
#undef READ
  py::class_<Result>(m, "Result")
    .def_readonly("action", &Result::action).def_readonly("reference", &Result::reference)
    .def_readonly("flags", &Result::flags).def_readonly("diagnostics", &Result::diagnostics)
    .def_readonly("message", &Result::message);
  py::class_<ServoCore>(m, "ServoCore")
    .def(py::init<std::shared_ptr<Kinematics>, Limits, Config>(), py::keep_alive<1, 2>())
    .def("step", &ServoCore::step, py::arg("state"), py::arg("command"), py::arg("dt"),
      py::arg("now_ns"), py::arg("collision") = std::nullopt, py::call_guard<py::gil_scoped_release>())
    .def("reset", &ServoCore::reset, py::call_guard<py::gil_scoped_release>());
}
