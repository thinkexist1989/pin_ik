#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <dlfcn.h>
#include <fstream>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  auto begin = reinterpret_cast<void (*)()>(dlsym(RTLD_DEFAULT, "allocation_begin"));
  auto end = reinterpret_cast<unsigned long (*)()>(dlsym(RTLD_DEFAULT, "allocation_end"));
  if (!begin || !end) { std::cerr << "Preload allocation_counter.so\n"; return 2; }
  std::ifstream input(argv[1]);
  const std::string xml((std::istreambuf_iterator<char>(input)), {});
  pinocchio::Model model;
  Eigen::VectorXd lo, hi;
  pinocchio::FrameIndex tip;
  PIN_IK::loadURDFModel(xml, "base", "tip", model, lo, hi, tip);
  pinocchio::Data data(model);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nv), seed = q, out(model.nv);
  pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, q));
  pinocchio::updateFramePlacements(model, data);
  const auto goal = data.oMf[tip];
  PIN_IK::ChainIkSolverPos_TL newton(model, lo, hi, tip, .02, 1e-7);
  NLOPT_IK::NLOPT_IK nlopt(model, lo, hi, tip, .02, 1e-7);
  PIN_IK::PIN_IK dual(model, lo, hi, tip, .02, 1e-7);
  for (bool exact : {true, false}) {
    seed = q;
    if (!exact) { seed[0] = .01; seed[3] = -.01; }
    for (int backend = 0; backend < 3; ++backend) {
      auto solve = [&] {
        if (backend == 0) return newton.CartToJnt(seed, goal, out);
        if (backend == 1) return nlopt.CartToJnt(seed, goal, out);
        return dual.CartToJnt(seed, goal, out);
      };
      for (int i = 0; i < 100; ++i) if (solve() < 0) return 1;
      unsigned long total = 0, maximum = 0;
      for (int i = 0; i < 1000; ++i) {
        begin();
        const int rc = solve();
        const auto allocations = end();
        if (rc < 0) return 1;
        total += allocations;
        maximum = std::max(maximum, allocations);
      }
      std::cout << "backend=" << backend << " exact=" << exact << " calls=1000 allocations="
                << total << " mean=" << total/1000.0 << " max=" << maximum << '\n';
    }
  }
}
