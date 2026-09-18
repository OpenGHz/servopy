#include "servo_py/servo.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace servo_py {
namespace {
void validate(const DifferentialIKRequest& r) {
  const int n = r.jacobian.cols();
  if (n < 1 || r.jacobian.rows() < 1 || r.task.size() != r.jacobian.rows() ||
      r.preferred_velocity.size() != n || r.lower.size() != n || r.upper.size() != n ||
      !r.jacobian.allFinite() || !r.task.allFinite() || !r.preferred_velocity.allFinite() ||
      !r.lower.allFinite() || !r.upper.allFinite() || (r.lower.array() > r.upper.array()).any() ||
      !std::isfinite(r.damping) || r.damping <= 0)
    throw std::invalid_argument("invalid differential IK request");
}
}

Vector DampedLeastSquares::solve(const DifferentialIKRequest& r) {
  validate(r);
  Eigen::JacobiSVD<Matrix> svd(r.jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Vector s = svd.singularValues();
  const Vector inverse = s.array() / (s.array().square() + r.damping * r.damping);
  return svd.matrixV() * inverse.asDiagonal() * svd.matrixU().transpose() * r.task + r.preferred_velocity;
}

BoxQPSolver::BoxQPSolver(int max_iterations, double tolerance)
    : max_iterations_(max_iterations), tolerance_(tolerance) {
  if (max_iterations < 1 || !std::isfinite(tolerance) || tolerance <= 0)
    throw std::invalid_argument("QP iteration limit and tolerance must be positive");
}

Vector BoxQPSolver::solve(const DifferentialIKRequest& r) {
  validate(r);
  const int n = r.jacobian.cols();
  const double regularization = r.damping * r.damping;
  const Matrix h = r.jacobian.transpose() * r.jacobian + regularization * Matrix::Identity(n, n);
  const Vector b = r.jacobian.transpose() * r.task + regularization * r.preferred_velocity;
  if (!h.allFinite() || !b.allFinite()) throw std::runtime_error("QP objective overflow");
  Vector x = h.ldlt().solve(b).cwiseMax(r.lower).cwiseMin(r.upper);
  std::vector<int> active(n, 0);
  for (int i = 0; i < n; ++i) {
    if (r.lower[i] == r.upper[i]) active[i] = 2;
    else if (x[i] <= r.lower[i]) active[i] = -1;
    else if (x[i] >= r.upper[i]) active[i] = 1;
  }
  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    const Vector gradient = h * x - b;
    std::vector<int> free;
    for (int i = 0; i < n; ++i) if (!active[i]) free.push_back(i);
    Vector direction = Vector::Zero(n);
    if (!free.empty()) {
      Matrix hf(free.size(), free.size());
      Vector gf(free.size());
      for (std::size_t i = 0; i < free.size(); ++i) {
        gf[i] = -gradient[free[i]];
        for (std::size_t j = 0; j < free.size(); ++j) hf(i, j) = h(free[i], free[j]);
      }
      const Vector delta = hf.ldlt().solve(gf);
      for (std::size_t i = 0; i < free.size(); ++i) direction[free[i]] = delta[i];
    }
    if (!direction.allFinite() || !x.allFinite()) throw std::runtime_error("QP factorization failed");
    if (direction.cwiseAbs().maxCoeff() <= tolerance_) {
      int release = -1;
      double worst = tolerance_;
      for (int i = 0; i < n; ++i) {
        const double violation = active[i] == -1 ? -gradient[i] : (active[i] == 1 ? gradient[i] : 0);
        if (violation > worst) { worst = violation; release = i; }
      }
      if (release < 0) return x;
      active[release] = 0;
    } else {
      double alpha = 1.0;
      int hit = -1, side = 0;
      for (int i : free) {
        double step = 2.0;
        if (direction[i] > 0) step = (r.upper[i] - x[i]) / direction[i];
        else if (direction[i] < 0) step = (r.lower[i] - x[i]) / direction[i];
        if (step <= alpha) { alpha = std::max(0.0, step); hit = i; side = direction[i] > 0 ? 1 : -1; }
      }
      x += alpha * direction;
      x = x.cwiseMax(r.lower).cwiseMin(r.upper);
      if (hit >= 0) active[hit] = side;
    }
  }
  throw std::runtime_error("QP iteration limit reached; no unchecked solution returned");
}
}  // namespace servo_py
