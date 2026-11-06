#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <fstream>
#include <iostream>
#include <limits>
#include <cstring>

namespace NLOPT_IK {
void constrainfuncm(unsigned, double*, unsigned, const double*, double*, void*);
struct SolverTestAccess {
  static void hook(void (*fn)(NLOPT_IK&, const char*)) { NLOPT_IK::test_hook = fn; }
  static void cancel(NLOPT_IK& s) { s.abort(); }
  static void expire(NLOPT_IK& s) { s.deadline = std::chrono::steady_clock::now(); }
  static void reset(NLOPT_IK& s) {
    s.reset(); s.progress = -3;
    s.deadline = std::chrono::steady_clock::time_point::max();
  }
  static double objective(NLOPT_IK& s, const std::vector<double>& x,
                          std::vector<double>& grad, OptType type) {
    if (type == Joint) {
      double result;
      constrainfuncm(1, &result, x.size(), x.data(), grad.data(), &s);
      return result;
    }
    return s.objective(x, grad.data(), type == SumSq ? &NLOPT_IK::cartSumSquaredError :
        type == L2 ? &NLOPT_IK::cartL2NormError : &NLOPT_IK::cartDQError);
  }
  static void prepare(NLOPT_IK& s, const pinocchio::SE3& goal) {
    reset(s); s.targetPose = goal; s.bounds.setZero();
  }
};
}
namespace PIN_IK {
struct SolverTestAccess {
  static void hook(void (*fn)(ChainIkSolverPos_TL&, const char*)) { ChainIkSolverPos_TL::test_hook = fn; }
  static void cancel(ChainIkSolverPos_TL& s) { s.abort(); }
  static void reset(ChainIkSolverPos_TL& s) { s.reset(); }
  static int expired(ChainIkSolverPos_TL& s, const Eigen::VectorXd& q,
                     const pinocchio::SE3& target, Eigen::VectorXd& out) {
    return s.CartToJntUntil(q, target, out, pinocchio::Motion::Zero(), std::chrono::steady_clock::now());
  }
};
struct WorkerTestAccess {
  static void hook(void (*fn)(PIN_IK&, const char*, unsigned)) { PIN_IK::test_hook = fn; }
  static void expire(PIN_IK& s) { s.deadline = std::chrono::steady_clock::now(); }
  static auto deadline(PIN_IK& s) { return s.deadline; }
};
}

namespace {
using Clock = std::chrono::steady_clock;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
int callbacks = 0, cancel_at = 0, expire_at = 0, fk_calls = 0, optimize_calls = 0;
bool fail_callback = false, expire_optimize = false, cancel_newton = false;
void nl_hook(NLOPT_IK::NLOPT_IK& s, const char* event) {
  if (!std::strcmp(event, "before_optimize")) {
    ++optimize_calls;
    if (expire_optimize) NLOPT_IK::SolverTestAccess::expire(s);
  }
  if (std::strcmp(event, "error_callback")) return;
  ++callbacks;
  if (fail_callback) throw std::bad_alloc();
  if (callbacks == cancel_at) NLOPT_IK::SolverTestAccess::cancel(s);
  if (callbacks == expire_at) NLOPT_IK::SolverTestAccess::expire(s);
}
void newton_hook(PIN_IK::ChainIkSolverPos_TL& s, const char*) {
  ++fk_calls;
  if (cancel_newton) PIN_IK::SolverTestAccess::cancel(s);
}
void dispatch_hook(PIN_IK::PIN_IK& s, const char* event, unsigned) {
  if (!std::strcmp(event, "before_dispatch")) PIN_IK::WorkerTestAccess::expire(s);
}
std::chrono::steady_clock::time_point cancel_time[2], complete_time[2], task_deadline;
void timing_hook(PIN_IK::PIN_IK& s, const char* event, unsigned index) {
  if (!std::strcmp(event, "before_dispatch")) task_deadline = PIN_IK::WorkerTestAccess::deadline(s);
  if (!std::strcmp(event, "before_cancel")) cancel_time[index] = Clock::now();
  if (!std::strcmp(event, "completed")) complete_time[index] = Clock::now();
}
pinocchio::SE3 fk(const pinocchio::Model& m, pinocchio::Data& d, pinocchio::FrameIndex tip,
                  const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(m, d, PIN_IK::toPinocchioConfiguration(m, q));
  return pinocchio::updateFramePlacement(m, d, tip);
}
void verify(const pinocchio::Model& m, pinocchio::Data& d, pinocchio::FrameIndex tip,
            const Eigen::VectorXd& q, const pinocchio::SE3& target) {
  require(q.allFinite(), "Non-finite result");
  require(pinocchio::log6(fk(m, d, tip, q).actInv(target)).toVector().norm() < 3e-6,
          "Returned solution fails FK validation");
}

// Validate both Jacobian paths against a numerical LOCAL pose derivative, including
// unrelated branches and fixed frames. Also measure the alternatives outside CTest gates.
void checkKinematics(pinocchio::Model model, pinocchio::FrameIndex tip, bool branched) {
  if (branched) {
    for (int i = 0; i < 12; ++i) {
      const auto joint = model.addJoint(0, pinocchio::JointModelRY(), pinocchio::SE3::Identity(),
                                        "branch" + std::to_string(i));
      model.addFrame(pinocchio::Frame("branch_frame" + std::to_string(i), joint, 0,
                                     pinocchio::SE3::Identity(), pinocchio::OP_FRAME));
    }
  }
  pinocchio::Data a(model), b(model), c(model);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv), native(model.nq);
  pinocchio::Data::Matrix6x ja(6, model.nv), jb(6, model.nv);
  double max_error = 0;
  for (int sample = 0; sample < 100; ++sample) {
    for (int i = 0; i < model.nv; ++i) q[i] = 0.4 * std::sin(sample * .1 + i);
    q[5] += 8 * M_PI;
    PIN_IK::toPinocchioConfiguration(model, q, native);
    require(native == PIN_IK::toPinocchioConfiguration(model, q), "Configuration conversion changed");
    pinocchio::forwardKinematics(model, a, native);
    pinocchio::updateFramePlacements(model, a);
    const auto pose = a.oMf[tip];
    pinocchio::computeJointJacobians(model, b, native);
    require(pinocchio::updateFramePlacement(model, b, tip).isApprox(pose, 1e-12), "Frame update changed pose");
    ja.setZero(); jb.setZero();
    pinocchio::computeFrameJacobian(model, a, native, tip, pinocchio::LOCAL, ja);
    pinocchio::getFrameJacobian(model, b, tip, pinocchio::LOCAL, jb);
    require(ja.isApprox(jb, 1e-12), "Jacobian paths disagree");
    pinocchio::forwardKinematics(model, b, native);
    pinocchio::computeJointJacobians(model, b);
    jb.setZero();
    pinocchio::getFrameJacobian(model, b, tip, pinocchio::LOCAL, jb);
    require(ja.isApprox(jb, 1e-12), "Reused FK Jacobian differs");
    for (int j = 0; j < model.nv; ++j) {
      auto perturbed = q;
      perturbed[j] += 1e-7;
      const Eigen::Matrix<double, 6, 1> numerical =
          pinocchio::log6(pose.actInv(fk(model, c, tip, perturbed))).toVector() / 1e-7;
      max_error = std::max(max_error, (numerical - jb.col(j)).norm());
    }
  }
  require(max_error < 2e-7, "LOCAL Jacobian differs from numerical derivative");
  for (int path = 0; path < 3; ++path) {
    const auto begin = Clock::now();
    for (int i = 0; i < 20000; ++i) {
      ja.setZero();
      if (path == 0) {
        pinocchio::forwardKinematics(model, a, native);
        pinocchio::updateFramePlacement(model, a, tip);
        pinocchio::computeFrameJacobian(model, a, native, tip, pinocchio::LOCAL, ja);
      } else if (path == 1) {
        pinocchio::computeJointJacobians(model, a, native);
        pinocchio::updateFramePlacement(model, a, tip);
        pinocchio::getFrameJacobian(model, a, tip, pinocchio::LOCAL, ja);
      } else {
        pinocchio::forwardKinematics(model, a, native);
        pinocchio::updateFramePlacement(model, a, tip);
        pinocchio::computeJointJacobians(model, a);
        pinocchio::getFrameJacobian(model, a, tip, pinocchio::LOCAL, ja);
      }
    }
    std::cout << "kinematics branched=" << branched << " path=" << path << " mean_us="
              << std::chrono::duration<double, std::micro>(Clock::now()-begin).count()/20000
              << " max_derivative_error=" << max_error << '\n';
  }
}
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Expected URDF path");
    std::ifstream input(argv[1]);
    const std::string xml((std::istreambuf_iterator<char>(input)), {});
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    pinocchio::FrameIndex tip;
    PIN_IK::loadURDFModel(xml, "base", "tip", model, lo, hi, tip);
    pinocchio::Data data(model);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv), out(model.nv);
    const auto target = fk(model, data, tip, q);
    NLOPT_IK::SolverTestAccess::hook(nl_hook);
    PIN_IK::SolverTestAccess::hook(newton_hook);

    for (auto type : {NLOPT_IK::SumSq, NLOPT_IK::L2, NLOPT_IK::DualQuat, NLOPT_IK::Joint}) {
      NLOPT_IK::NLOPT_IK solver(model, lo, hi, tip, .1, 1e-7, type);
      // Initialize all mode-specific state, then test each callback exit explicitly.
      require(solver.CartToJnt(q, target, out) > 0, "NLopt exact solve failed");
      verify(model, data, tip, out, target);
      for (int stop = 0; stop < 4; ++stop) {
        NLOPT_IK::SolverTestAccess::prepare(solver, target);
        std::vector<double> x(model.nv, .1), grad(model.nv, std::numeric_limits<double>::quiet_NaN());
        callbacks = 0;
        cancel_at = stop < 2 ? stop + 1 : 0;
        expire_at = stop >= 2 ? stop - 1 : 0;
        const double value = NLOPT_IK::SolverTestAccess::objective(solver, x, grad, type);
        require(std::isfinite(value), "Stopped callback left error undefined");
        require(callbacks == (stop % 2 + 1), "Difference loop continued after stop");
        for (double g : grad) require(g == 0, "Stopped callback used invalid gradient output");
      }
      cancel_at = expire_at = 0;
      NLOPT_IK::SolverTestAccess::prepare(solver, target);
      callbacks = 0;
      std::vector<double> exact(model.nv, 0), grad(model.nv, 123);
      require(std::isfinite(NLOPT_IK::SolverTestAccess::objective(solver, exact, grad, type)), "Success callback undefined");
      require(callbacks == 1, "Gradient continued after successful base evaluation");
      for (double g : grad) require(g == 0, "Success gradient not initialized");

      NLOPT_IK::SolverTestAccess::reset(solver);
      fail_callback = true;
      bool propagated = false;
      try { solver.CartToJnt(q, target, out); } catch (const std::bad_alloc&) { propagated = true; }
      fail_callback = false;
      require(propagated, "NLopt swallowed callback allocation failure");
      require(solver.CartToJnt(q, target, out) > 0, "NLopt failed after exception");
      expire_optimize = true; callbacks = optimize_calls = 0;
      require(solver.CartToJnt(q, target, out) == -3, "Expired optimize accepted a result");
      require(optimize_calls == 1 && callbacks == 0, "NLopt started with expired budget");
      expire_optimize = false;
      require(solver.CartToJnt(q, target, out) > 0, "NLopt deadline not reset");
      for (int sample = 0; sample < 20; ++sample) {
        Eigen::VectorXd desired = q;
        for (int j = 0; j < model.nv; ++j) desired[j] = .2 * std::sin(sample*.1+j);
        desired[5] += 8 * M_PI;
        auto seed = desired;
        seed[0] += .01; seed[3] -= .01;
        const auto goal = fk(model, data, tip, desired);
        require(solver.CartToJnt(seed, goal, out, pinocchio::Motion::Zero(), desired) > 0,
                "NLopt perturbed-seed solve failed");
        verify(model, data, tip, out, goal);
        for (int j = 0; j < model.nv; ++j)
          require(out[j] >= lo[j]-1e-8 && out[j] <= hi[j]+1e-8, "NLopt limit violation");
      }
    }

    PIN_IK::ChainIkSolverPos_TL newton(model, lo, hi, tip, .1, 1e-7, true, true);
    fk_calls = 0;
    require(PIN_IK::SolverTestAccess::expired(newton, q, target, out) == -3 && fk_calls == 0,
            "Newton started after deadline");
    cancel_newton = true;
    require(newton.CartToJnt(q, target, out) == -3, "Newton ignored cancellation");
    cancel_newton = false;
    PIN_IK::SolverTestAccess::reset(newton);
    auto seed = q; seed[0] = .1; seed[3] = -.2;
    require(newton.CartToJnt(seed, target, out) > 0, "Newton recovery failed");
    verify(model, data, tip, out, target);

    PIN_IK::WorkerTestAccess::hook(dispatch_hook);
    {
      PIN_IK::PIN_IK solver(model, lo, hi, tip, .02, 1e-7);
      callbacks = fk_calls = optimize_calls = 0;
      require(solver.CartToJnt(q, target, out) == -3 && out == q, "Expired task result incorrect");
      require(callbacks == 0 && fk_calls == 0 && optimize_calls == 0, "Late worker started computing");
    }
    PIN_IK::WorkerTestAccess::hook(nullptr);
    NLOPT_IK::SolverTestAccess::hook(nullptr);
    PIN_IK::SolverTestAccess::hook(nullptr);
    PIN_IK::WorkerTestAccess::hook(timing_hook);
    {
      PIN_IK::PIN_IK solver(model, lo, hi, tip, .00055, 1e-7);
      auto unreachable = target;
      unreachable.translation().setConstant(10);
      double total = 0, maximum = 0, cancel_max = 0;
      for (int i = 0; i < 1000; ++i) {
        require(solver.CartToJnt(q, unreachable, out) == -3 && out == q, "Deadline returned false solution");
        const auto finished = std::max(complete_time[0], complete_time[1]);
        const double tail = std::max(0.0, std::chrono::duration<double, std::micro>(finished-task_deadline).count());
        total += tail; maximum = std::max(maximum, tail);
        cancel_max = std::max(cancel_max, std::chrono::duration<double, std::micro>(
            finished-std::min(cancel_time[0], cancel_time[1])).count());
      }
      std::cout << "deadline_tail calls=1000 mean_us=" << total/1000 << " max_us=" << maximum
                << " cancel_to_complete_max_us=" << cancel_max << '\n';
    }
    PIN_IK::WorkerTestAccess::hook(nullptr);
    checkKinematics(model, tip, false);
    checkKinematics(model, tip, true);
    std::cout << "Deadline, cancellation, callback outputs, exception recovery and kinematics passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n'; return 1;
  }
}
