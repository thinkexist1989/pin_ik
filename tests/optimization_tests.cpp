#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <fstream>
#include <iostream>
#include <cstring>

namespace NLOPT_IK {
struct SolverTestAccess { static double draw(NLOPT_IK& s) { return s.fRand(-2, 3); } };
}
namespace PIN_IK {
struct SolverTestAccess { static double draw(ChainIkSolverPos_TL& s) { return s.fRand(-2, 3); } };
struct WorkerTestAccess {
  static double draw(PIN_IK& s, unsigned stream) {
    if (stream < 2) return s.random[stream].uniform(-2, 3);
    if (stream == 2) return SolverTestAccess::draw(*s.iksolver);
    return NLOPT_IK::SolverTestAccess::draw(*s.nl_solver);
  }
  static void hook(void (*fn)(PIN_IK&, const char*, unsigned)) { PIN_IK::test_hook = fn; }
};
}
namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
struct Gate {
  std::mutex mutex;
  std::condition_variable cv;
  bool scoring = false, peer_submitted = false;
  int held_worker = 0;
};
Gate* gate = nullptr;
void hook(PIN_IK::PIN_IK&, const char* event, unsigned index) {
  if (std::strcmp(event, "before_score") && std::strcmp(event, "before_solve") &&
      std::strcmp(event, "before_result")) return;
  std::unique_lock<std::mutex> lock(gate->mutex);
  if (!std::strcmp(event, "before_score") && static_cast<int>(index) == gate->held_worker) {
    gate->scoring = true;
    gate->cv.notify_all();
    // The CTest process timeout is the watchdog. Plain wait also works with
    // older TSan runtimes that do not intercept pthread_cond_clockwait.
    gate->cv.wait(lock, [] { return gate->peer_submitted; });
  }
  if (!std::strcmp(event, "before_solve") && static_cast<int>(index) != gate->held_worker) {
    gate->cv.wait(lock, [] { return gate->scoring; });
  }
  if (!std::strcmp(event, "before_result") && static_cast<int>(index) != gate->held_worker) {
    gate->peer_submitted = true;
    gate->cv.notify_all();
  }
}
double score(const pinocchio::Model& model, pinocchio::FrameIndex tip,
             const Eigen::VectorXd& lo, const Eigen::VectorXd& hi,
             const Eigen::VectorXd& seed, const Eigen::VectorXd& q, int mode) {
  if (mode < 2) return (seed-q).squaredNorm();
  pinocchio::Data data(model);
  pinocchio::Data::Matrix6x jacobian = pinocchio::Data::Matrix6x::Zero(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, PIN_IK::toPinocchioConfiguration(model, q), tip,
                                  pinocchio::LOCAL_WORLD_ALIGNED, jacobian);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  const auto& values = svd.singularValues();
  double value = mode == 2 ? values.prod() :
                 mode == 3 ? values.minCoeff()/values.maxCoeff() : values.minCoeff();
  double penalty = 1;
  const auto types = PIN_IK::jointTypes(model);
  for (int j = 0; j < model.nv; ++j) if (types[j] != PIN_IK::Continuous)
    penalty *= ((q[j]-lo[j])*(hi[j]-q[j]))/((hi[j]-lo[j])*(hi[j]-lo[j]));
  return std::max(0.0, 1-std::exp(-penalty))*value;
}
}

int main(int argc, char** argv) {
  try {
    require(argc == 2, "Expected URDF path");
    std::ifstream input(argv[1]);
    std::string xml((std::istreambuf_iterator<char>(input)), {});
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    pinocchio::FrameIndex tip;
    PIN_IK::loadURDFModel(xml, "base", "tip", model, lo, hi, tip);
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(model.nv), out(model.nv);
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, seed));
    const auto target = pinocchio::updateFramePlacement(model, data, tip);
    {
      PIN_IK::PIN_IK a(model,lo,hi,tip), b(model,lo,hi,tip);
      a.setRandomSeed(42); b.setRandomSeed(42);
      bool distinct = false;
      for (int i = 0; i < 1000; ++i) {
        double first = 0;
        for (unsigned stream = 0; stream < 4; ++stream) {
          const double x = PIN_IK::WorkerTestAccess::draw(a,stream);
          // Unrelated process-global calls cannot perturb a worker's stream.
          std::srand(i); std::rand();
          const double y = PIN_IK::WorkerTestAccess::draw(b,stream);
          require(x == y && x >= -2 && x < 3, "Random stream not reproducible or outside range");
          if (stream == 0) first = x;
          else distinct |= x != first;
        }
      }
      require(distinct, "Worker streams are identical");
      a.setLimits(lo,hi);
      for (unsigned stream = 0; stream < 4; ++stream)
        require(PIN_IK::WorkerTestAccess::draw(a,stream) == PIN_IK::WorkerTestAccess::draw(b,stream),
                "setLimits unexpectedly reset the random sequence");
      a.setRandomSeed(7); b.setRandomSeed(7);
      std::exception_ptr error[2];
      auto run = [&](PIN_IK::PIN_IK& s, int index) {
        try {
          Eigen::VectorXd result(model.nv);
          for (int i = 0; i < 100; ++i)
            require(s.CartToJnt(seed,target,result) > 0, "Parallel seeded instance failed");
        } catch (...) { error[index] = std::current_exception(); }
      };
      std::thread one(run,std::ref(a),0), two(run,std::ref(b),1);
      one.join(); two.join();
      for (auto& e : error) if (e) std::rethrow_exception(e);
    }
    // Hold either worker just before scoring. Its peer must still acquire the
    // result lock and submit; no probabilistic sleep-based interleaving.
    for (int held : {0,1}) {
      Gate scheduling; scheduling.held_worker = held; gate = &scheduling;
      PIN_IK::WorkerTestAccess::hook(hook);
      {
        PIN_IK::PIN_IK solver(model,lo,hi,tip,.05,1e-7,PIN_IK::Manip1);
        solver.setRandomSeed(42);
        require(solver.CartToJnt(seed,target,out) > 0, "Gated scoring solve failed");
        require(scheduling.scoring && scheduling.peer_submitted, "Scoring gate not exercised");
      }
      PIN_IK::WorkerTestAccess::hook(nullptr);
    }
    {
      PIN_IK::PIN_IK solver(model,lo,hi,tip,.01,1e-7);
      solver.setRandomSeed(42);
      for (int mode = 0; mode < 5; ++mode) {
        solver.SetSolveType(static_cast<PIN_IK::SolveType>(mode));
        const int count = solver.CartToJnt(seed,target,out);
        require(count > 0, "Mode failed");
        std::vector<Eigen::VectorXd> solutions;
        std::vector<std::pair<double,unsigned>> errors;
        require(solver.getSolutions(solutions,errors), "Missing scored candidates");
        require(count == static_cast<int>(solutions.size()) && errors.size() == solutions.size(), "Candidate count changed");
        for (size_t i = 0; i < errors.size(); ++i) {
          require(errors[i].second < solutions.size(), "Invalid score index");
          const auto& q = solutions[errors[i].second];
          pinocchio::forwardKinematics(model,data,PIN_IK::toPinocchioConfiguration(model,q));
          const auto pose = pinocchio::updateFramePlacement(model,data,tip);
          require(pinocchio::log6(pose.actInv(target)).toVector().norm() < 3e-6, "Invalid candidate FK");
          const double expected = score(model,tip,lo,hi,seed,q,mode);
          require(std::isfinite(errors[i].first) && std::abs(expected-errors[i].first) < 1e-12*(1+std::abs(expected)),
                  "Parallel score differs from independent reference");
          if (i) require(mode < 2 ? errors[i-1] <= errors[i] : errors[i-1] >= errors[i], "Score order changed");
        }
        require(out == solutions[errors.front().second], "Returned candidate is not the best score");
      }
    }
    std::cout << "Independent random streams, reseeding, limits, concurrent scoring and candidate scores passed\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
