#include "servo_py/servo.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
  using namespace servo_py;
  Joint joint; joint.name = "slide"; joint.type = JointType::PRISMATIC;
  joint.axis = Eigen::Vector3d::UnitX();
  Limits limits{Vector::Constant(1, -1), Vector::Constant(1, 1),
                Vector::Constant(1, 1), Vector::Constant(1, 2), Vector::Constant(1, .01)};
  auto model = std::make_shared<SerialChain>(std::vector<Joint>{joint}, limits);
  Config config; config.task_axes = {0};
  ServoCore servo(model, limits, config);
  State state{Vector::Zero(1), Vector::Zero(1), 0};
  for (int k = 0; k < 100; ++k) {
    Command command;
    command.type = k < 50 ? CommandType::JOINT_JOG : CommandType::STOP;
    command.joint_velocity = Vector::Constant(1, .3);
    command.stamp_ns = k * 10000000LL;
    const auto result = servo.step(state, command, .01, command.stamp_ns);
    if (!result.reference || result.action == Action::REJECT)
      throw std::runtime_error(result.message);
    const auto& ref = *result.reference;
    if (std::abs(ref.ddq[0]) > 2.000000001 || std::abs(ref.q[0]) > .99)
      throw std::runtime_error("reference constraints failed");
    state = {ref.q, ref.dq, ref.stamp_ns};
  }
  if (std::abs(state.dq[0]) > 1e-10) throw std::runtime_error("stop did not finish");
  Result position_result;
  for (int k = 100; k < 800; ++k) {
    Command position;
    position.type = CommandType::JOINT_POSITION;
    position.joint_position = Vector::Constant(1, .45);
    position.stamp_ns = k * 10000000LL;
    position_result = servo.step(state, position, .01, position.stamp_ns);
    if (!position_result.reference) throw std::runtime_error(position_result.message);
    const auto& ref = *position_result.reference;
    if (std::abs(ref.ddq[0]) > 2.000000001 || std::abs(ref.dq[0]) > 1.000000001)
      throw std::runtime_error("joint position limits failed");
    state = {ref.q, ref.dq, ref.stamp_ns};
  }
  if (position_result.action != Action::HOLD || !(position_result.flags & GOAL_REACHED) ||
      std::abs(state.q[0] - .45) > 1.1e-4)
    throw std::runtime_error("joint position target did not converge");
  DifferentialIKRequest request;
  request.jacobian = Matrix::Ones(1, 2);
  request.task = Vector::Ones(1);
  request.lower = Vector::Constant(2, -1);
  request.upper = Vector::Ones(2); request.upper[0] = .1;
  request.preferred_velocity = Vector::Zero(2); request.damping = .01;
  const Vector solved = BoxQPSolver().solve(request);
  if (std::abs(solved[0] - .1) > 1e-8 || solved.sum() < .999)
    throw std::runtime_error("bounded QP failed to redistribute joint motion");
  ServoCore qp_servo(model, limits, config, std::make_shared<BoxQPSolver>());
  Command twist; twist.type = CommandType::TWIST; twist.linear[0] = .1;
  const auto result = qp_servo.step(State{Vector::Zero(1), Vector::Zero(1), 0}, twist, .01, 0);
  if (!result.reference || result.reference->dq[0] <= 0 ||
      (qp_servo.sample_reference(.01).q - result.reference->q).norm() > 1e-10)
    throw std::runtime_error("C++ solver injection or reference sampling failed");
  std::cout << "Standalone C++ servo: PASS\n";
}
