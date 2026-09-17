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
  std::cout << "Standalone C++ servo: PASS\n";
}
