#include <pin_ik/nlopt_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <fstream>
#include <iostream>
#include <random>

namespace NLOPT_IK {
struct SolverTestAccess {
  static void prepare(NLOPT_IK& s, const pinocchio::Motion& bounds) {
    s.reset(); s.progress = -3; s.bounds = bounds;
    s.deadline = std::chrono::steady_clock::time_point::max();
  }
  static auto function(OptType type) {
    return type == DualQuat ? &NLOPT_IK::cartDQError :
           type == L2 ? &NLOPT_IK::cartL2NormError : &NLOPT_IK::cartSumSquaredError;
  }
  static double objective(NLOPT_IK& s, const std::vector<double>& x, double* grad, OptType type) {
    return s.objective(x, grad, function(type));
  }
  static double value(NLOPT_IK& s, const std::vector<double>& x, OptType type) {
    double out;
    (s.*function(type))(x, &out);
    return out;
  }
};
}

int main(int argc, char** argv) {
  try {
    if (argc < 2) throw std::runtime_error("Expected URDF path [--diagnostic]");
    const bool diagnostic = argc == 3 && std::string(argv[2]) == "--diagnostic";
    std::ifstream input(argv[1]);
    std::string xml((std::istreambuf_iterator<char>(input)), {});
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    pinocchio::FrameIndex tip;
    PIN_IK::loadURDFModel(xml, "base", "tip", model, lo, hi, tip);
    pinocchio::Data data(model);
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> dist(-.6, .6);
    double worst = 0;
    for (auto type : {NLOPT_IK::SumSq, NLOPT_IK::L2, NLOPT_IK::DualQuat, NLOPT_IK::Joint}) {
      double max_error = 0, scale_sum = 0;
      NLOPT_IK::NLOPT_IK solver(model, lo, hi, tip, .1, 1e-12, type);
      for (int sample = 0; sample < 150; ++sample) {
        Eigen::VectorXd desired(model.nv), output(model.nv);
        std::vector<double> x(model.nv), gradient(model.nv);
        for (int j = 0; j < model.nv; ++j) { desired[j] = dist(rng); x[j] = dist(rng); }
        desired[5] += 8*M_PI; x[5] += 8*M_PI;
        pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, desired));
        const auto target = pinocchio::updateFramePlacement(model, data, tip);
        if (solver.CartToJnt(desired, target, output) < 0) throw std::runtime_error("Gradient fixture setup failed");
        auto bounds = pinocchio::Motion::Zero();
        // Active/inactive residual components, kept away from discontinuous boundaries.
        if (sample % 2) bounds.linear().setConstant(2.0);
        NLOPT_IK::SolverTestAccess::prepare(solver, bounds);
        NLOPT_IK::SolverTestAccess::objective(solver, x, gradient.data(), type);
        Eigen::VectorXd reference(model.nv);
        for (int j = 0; j < model.nv; ++j) {
          const double original = x[j], step = 1e-6;
          x[j] = original + step;
          NLOPT_IK::SolverTestAccess::prepare(solver, bounds);
          const double plus = NLOPT_IK::SolverTestAccess::value(solver, x, type);
          x[j] = original - step;
          NLOPT_IK::SolverTestAccess::prepare(solver, bounds);
          const double minus = NLOPT_IK::SolverTestAccess::value(solver, x, type);
          x[j] = original;
          reference[j] = (plus-minus)/(2*step);
        }
        const Eigen::Map<Eigen::VectorXd> actual(gradient.data(), gradient.size());
        const double error = (actual-reference).norm()/(1+reference.norm());
        if (!actual.allFinite() || !std::isfinite(error)) throw std::runtime_error("Invalid gradient");
        max_error = std::max(max_error, error);
        scale_sum += actual.dot(reference)/reference.squaredNorm();
      }
      std::cout << "type=" << type << " samples=150 max_normalized_error=" << max_error
                << " mean_gradient_scale=" << scale_sum/150 << '\n';
      worst = std::max(worst, max_error);
    }
    if (!diagnostic && worst > 5e-5) throw std::runtime_error("Gradient does not match central numerical derivative");
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
