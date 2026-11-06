#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

namespace NLOPT_IK {
struct SolverTestAccess {
  // NLopt exposes objective evaluations for its last optimize() call, not the
  // total SLSQP iteration count across random restarts.
  static int evaluations(NLOPT_IK& s) { return s.opt.get_numevals(); }
};
}
template<class Solver>
auto seedSolver(Solver& solver, int) -> decltype(solver.setRandomSeed(42), void()) { solver.setRandomSeed(42); }
template<class Solver> void seedSolver(Solver&, long) { std::srand(42); }

int main(int argc, char** argv) {
  try {
    if (argc != 5) throw std::runtime_error("Expected URDF output.csv samples timeout_seconds");
    std::ifstream input(argv[1]);
    std::string xml((std::istreambuf_iterator<char>(input)), {});
    pinocchio::Model model;
    Eigen::VectorXd lo, hi;
    pinocchio::FrameIndex tip;
    PIN_IK::loadURDFModel(xml,"base","tip",model,lo,hi,tip);
    const auto types = PIN_IK::jointTypes(model);
    pinocchio::Data data(model);
    const int count = std::stoi(argv[3]);
    const double timeout = std::stod(argv[4]);
    std::ofstream output(argv[2]);
    if (!output) throw std::runtime_error("Cannot create CSV");
    output << "backend,scenario,mode,case,rc,us,last_optimize_evaluations,candidates,best_score,position_error,rotation_error\n"
           << std::setprecision(17);
    for (int backend = 0; backend < 2; ++backend) {
      for (int mode = 0; mode < (backend == 0 ? 1 : 5); ++mode) {
        for (int scenario = 0; scenario < 2; ++scenario) {
          NLOPT_IK::NLOPT_IK nl(model,lo,hi,tip,timeout,1e-6);
          PIN_IK::PIN_IK dual(model,lo,hi,tip,timeout,1e-6,static_cast<PIN_IK::SolveType>(mode));
          std::mt19937_64 rng(731);
          std::uniform_real_distribution<double> random(-.7,.7);
          for (int sample = -30; sample < count; ++sample) {
            Eigen::VectorXd desired(model.nv), seed(model.nv), result(model.nv);
            for (int j = 0; j < model.nv; ++j) {
              desired[j] = random(rng);
              seed[j] = scenario ? random(rng) : desired[j]+(j%2 ? -.01 : .01);
            }
            for (int j = 0; j < model.nv; ++j) if (types[j] == PIN_IK::Continuous) {
              desired[j] += 8*M_PI; seed[j] += 8*M_PI;
            }
            pinocchio::forwardKinematics(model,data,PIN_IK::toPinocchioConfiguration(model,desired));
            const auto target = pinocchio::updateFramePlacement(model,data,tip);
            seedSolver(nl,0); seedSolver(dual,0);
            const auto start = std::chrono::steady_clock::now();
            const int rc = backend == 0 ? nl.CartToJnt(seed,target,result) : dual.CartToJnt(seed,target,result);
            const double us = std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
            // A timeout before optimize() may leave its previous counter intact.
            int evaluations = backend == 0 && rc >= 0 ? NLOPT_IK::SolverTestAccess::evaluations(nl) : -1;
            std::vector<Eigen::VectorXd> solutions;
            std::vector<std::pair<double,unsigned>> scores;
            if (backend) dual.getSolutions(solutions,scores);
            double position = 0, rotation = 0;
            if (rc >= 0) {
              pinocchio::forwardKinematics(model,data,PIN_IK::toPinocchioConfiguration(model,result));
              const auto actual = pinocchio::updateFramePlacement(model,data,tip);
              position = (actual.translation()-target.translation()).norm();
              rotation = Eigen::AngleAxisd(actual.rotation().transpose()*target.rotation()).angle();
              if (!result.allFinite() || position > 5e-6 || rotation > 5e-6) throw std::runtime_error("Invalid solution FK");
              for (int j = 0; j < model.nv; ++j)
                if (result[j] < lo[j]-1e-8 || result[j] > hi[j]+1e-8) throw std::runtime_error("Joint limit violation");
            }
            if (sample >= 0) output << backend << ',' << scenario << ',' << mode << ',' << sample << ',' << rc << ',' << us
                << ',' << evaluations << ',' << solutions.size() << ',' << (scores.empty()?0:scores.front().first)
                << ',' << position << ',' << rotation << '\n';
          }
        }
      }
    }
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
