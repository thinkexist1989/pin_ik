#include <pin_ik/pin_ik.hpp>
#include <pin_ik/urdf.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <system_error>

namespace PIN_IK {
struct WorkerTestAccess {
  static void hook(void (*fn)(PIN_IK&, const char*, unsigned)) { PIN_IK::test_hook = fn; }
  static std::array<std::thread::id, 2> ids(PIN_IK& solver) {
    return {solver.task1.get_id(), solver.task2.get_id()};
  }
  static bool complete(PIN_IK& solver) {
    std::lock_guard<std::mutex> lock(solver.worker_mutex);
    return solver.completed_workers == 2;
  }
  static void notify(PIN_IK& solver) { solver.work_cv.notify_all(); }
};
}

namespace {
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

struct Hooks {
  std::mutex mutex;
  std::condition_variable cv;
  int delayed = -1;
  bool release = false;
  bool fail_start = false;
  bool fail_result = false;
  unsigned entered[2] = {0, 0};
  unsigned finished[2] = {0, 0};
};
Hooks* hooks = nullptr; // Set only when no solver threads exist.

void hook(PIN_IK::PIN_IK&, const char* event, unsigned index) {
  std::unique_lock<std::mutex> lock(hooks->mutex);
  if (!std::strcmp(event, "before_start") && hooks->fail_start && index == 1)
    throw std::system_error(std::make_error_code(std::errc::resource_unavailable_try_again));
  if (!std::strcmp(event, "before_solve")) {
    ++hooks->entered[index];
    if (static_cast<int>(index) == hooks->delayed)
      hooks->cv.wait(lock, [] { return hooks->release; });
  }
  if (!std::strcmp(event, "before_result") && hooks->fail_result) {
    hooks->fail_result = false;
    hooks->release = true;
    hooks->cv.notify_all();
    throw std::runtime_error("injected result failure");
  }
  if (!std::strcmp(event, "after_solve")) {
    ++hooks->finished[index];
    if (static_cast<int>(index) != hooks->delayed) {
      hooks->release = true;
      hooks->cv.notify_all();
    }
  }
}

pinocchio::SE3 fk(const pinocchio::Model& model, pinocchio::Data& data,
                  pinocchio::FrameIndex tip, const Eigen::VectorXd& q) {
  pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, q));
  pinocchio::updateFramePlacements(model, data);
  return data.oMf[tip];
}

void verify(const pinocchio::Model& model, pinocchio::Data& data, pinocchio::FrameIndex tip,
            const Eigen::VectorXd& out, const pinocchio::SE3& goal,
            const Eigen::VectorXd& lo, const Eigen::VectorXd& hi,
            const pinocchio::Motion& bounds = pinocchio::Motion::Zero()) {
  require(out.allFinite(), "Non-finite solution");
  for (int j = 0; j < out.size(); ++j)
    require(out[j] >= lo[j] - 1e-8 && out[j] <= hi[j] + 1e-8, "Joint limit violation");
  const auto error = pinocchio::log6(fk(model, data, tip, out).actInv(goal));
  for (int j = 0; j < 6; ++j)
    require(std::abs(error.toVector()[j]) < std::abs(bounds.toVector()[j]) + 2e-6,
            "Stale target/bounds or incorrect solution");
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
    const auto goal = fk(model, data, tip, q);

    // Constructor rollback: the first real thread has started before the injected failure.
    Hooks startup;
    startup.fail_start = true;
    hooks = &startup;
    PIN_IK::WorkerTestAccess::hook(hook);
    bool rejected = false;
    try { PIN_IK::PIN_IK solver(model, lo, hi, tip); }
    catch (const std::system_error&) { rejected = true; }
    require(rejected, "Second-thread startup failure not propagated");

    // Hold either worker until its peer finishes, including late pickup after cancellation.
    for (int delayed : {0, 1}) {
      Hooks scheduling;
      scheduling.delayed = delayed;
      hooks = &scheduling;
      {
        PIN_IK::PIN_IK solver(model, lo, hi, tip, 0.1, 1e-7);
        const auto ids = PIN_IK::WorkerTestAccess::ids(solver);
        for (int iteration = 0; iteration < 50; ++iteration) {
          for (int wake = 0; wake < 10; ++wake) PIN_IK::WorkerTestAccess::notify(solver);
          {
            std::lock_guard<std::mutex> lock(scheduling.mutex);
            scheduling.release = false;
          }
          require(solver.CartToJnt(q, goal, out) > 0, "Gated solve failed");
          require(PIN_IK::WorkerTestAccess::complete(solver), "Returned before both workers finished");
          require(PIN_IK::WorkerTestAccess::ids(solver) == ids, "Worker identity changed");
          verify(model, data, tip, out, goal, lo, hi);
        }
      }
      require(scheduling.entered[0] == 50 && scheduling.entered[1] == 50 &&
              scheduling.finished[0] == 50 && scheduling.finished[1] == 50,
              "Spurious wake or missed/duplicate task");
    }

    Hooks exceptions;
    exceptions.fail_result = true;
    hooks = &exceptions;
    {
      PIN_IK::PIN_IK solver(model, lo, hi, tip, 0.1, 1e-7);
      bool propagated = false;
      try { solver.CartToJnt(q, goal, out); }
      catch (const std::runtime_error& error) {
        propagated = std::string(error.what()) == "injected result failure";
      }
      require(propagated && PIN_IK::WorkerTestAccess::complete(solver), "Worker failure lost");
      require(solver.CartToJnt(q, goal, out) > 0, "Exception left result lock held or state stale");
      verify(model, data, tip, out, goal, lo, hi);
    }
    PIN_IK::WorkerTestAccess::hook(nullptr);
    hooks = nullptr;

    // Exercise task reuse, configuration swaps and all result modes on actual solvers.
    {
      PIN_IK::PIN_IK solver(model, lo, hi, tip, 0.02, 1e-7);
      const auto ids = PIN_IK::WorkerTestAccess::ids(solver);
      for (int mode = 0; mode < 5; ++mode) {
        solver.SetSolveType(static_cast<PIN_IK::SolveType>(mode));
        require(solver.CartToJnt(q, goal, out) > 0, "Mode change failed");
        verify(model, data, tip, out, goal, lo, hi);
        std::vector<Eigen::VectorXd> solutions;
        std::vector<std::pair<double, unsigned>> errors;
        require(solver.getSolutions(solutions, errors), "Missing result cache");
        require(errors.size() == solutions.size(), "Score/solution count mismatch");
        for (size_t j = 1; j < errors.size(); ++j)
          require(mode >= 2 ? errors[j-1] >= errors[j] : errors[j-1] <= errors[j], "Wrong score order");
      }
      solver.SetSolveType(PIN_IK::Speed);
      Eigen::VectorXd narrow_lo = lo, narrow_hi = hi;
      narrow_lo[0] = -0.4; narrow_hi[0] = 0.4;
      require(solver.setLimits(narrow_lo, narrow_hi), "setLimits failed");
      Eigen::VectorXd invalid_lo = narrow_lo;
      invalid_lo[0] = 0.5;
      rejected = false;
      try { solver.setLimits(invalid_lo, narrow_hi); }
      catch (const std::invalid_argument&) { rejected = true; }
      require(rejected, "Invalid limit accepted");
      Eigen::VectorXd actual_lo, actual_hi;
      solver.getLimits(actual_lo, actual_hi);
      require(actual_lo == narrow_lo && actual_hi == narrow_hi, "Failed setLimits changed configuration");
      Eigen::VectorXd outside = q;
      outside[0] = 0.7;
      const auto outside_target = fk(model, data, tip, outside);
      require(solver.CartToJnt(q, outside_target, out) == -3, "New solver ignored narrower limits");
      require(solver.setLimits(lo, hi), "Limit restoration failed");
      require(solver.CartToJnt(q, outside_target, out) > 0, "Restored limits not used by workers");
      verify(model, data, tip, out, outside_target, lo, hi);
      require(PIN_IK::WorkerTestAccess::ids(solver) == ids, "setLimits recreated workers");

      for (int i = 0; i < 100000; ++i) {
        q[0] = 0.2 * std::sin(i * 0.01);
        q[3] = 0.3 * std::cos(i * 0.02);
        q[5] = 8 * M_PI + 0.4 * std::sin(i * 0.03);
        auto target = fk(model, data, tip, q);
        auto bounds = pinocchio::Motion::Zero();
        if (i % 1000 == 0) {
          target.translation().setConstant(10);
          require(solver.CartToJnt(q, target, out) == -3 && out == q, "Timeout result incorrect");
          std::vector<Eigen::VectorXd> results;
          require(!solver.getSolutions(results) && results.empty(), "Stale successful result after timeout");
        } else if (i % 25 == 0) {
          require(solver.CartToJnt(Eigen::VectorXd(2), target, out) == -1, "Invalid seed accepted");
        } else {
          if (i % 3 == 0) {
            bounds.linear().setConstant(0.001);
            target.translation()[0] += 0.0001;
          }
          require(solver.CartToJnt(q, target, out, bounds) > 0, "Repeated solve failed");
          verify(model, data, tip, out, target, lo, hi, bounds);
          require(std::abs(out[5] - q[5]) <= M_PI, "Continuous winding changed");
        }
      }
      require(PIN_IK::WorkerTestAccess::ids(solver) == ids, "Workers replaced during stress test");
    }

    for (int i = 0; i < 30; ++i) {
      PIN_IK::PIN_IK solver(model, lo, hi, tip, 0.02, 1e-7);
      if (i % 2) require(solver.CartToJnt(q, fk(model, data, tip, q), out) > 0, "Lifecycle solve failed");
    }
    std::exception_ptr parallel_errors[2];
    auto parallel = [&](int index) {
      try {
        PIN_IK::PIN_IK solver(model, lo, hi, tip, 0.02, 1e-7);
        pinocchio::Data local_data(model);
        Eigen::VectorXd seed = q, result(model.nv);
        for (int i = 0; i < 200; ++i) {
          seed[0] = 0.2 * std::sin(i + index);
          const auto target = fk(model, local_data, tip, seed);
          require(solver.CartToJnt(seed, target, result) > 0, "Concurrent instances failed");
          verify(model, local_data, tip, result, target, lo, hi);
        }
      } catch (...) { parallel_errors[index] = std::current_exception(); }
    };
    std::thread a(parallel, 0), b(parallel, 1);
    a.join(); b.join();
    for (const auto& error : parallel_errors) if (error) std::rethrow_exception(error);
    std::cout << "Worker lifecycle, gated scheduling, rollback, exceptions, modes, limits, "
                 "100000 mixed calls and parallel instances passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
