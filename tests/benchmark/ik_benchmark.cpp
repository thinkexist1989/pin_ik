// Compile this identical workload into separate executables: the two libraries
// contain overlapping NLOPT_IK symbols and must not be linked into one process.
#ifdef BENCH_KDL
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#else
#include <pin_ik/pin_ik.hpp>
#endif
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>

#ifdef BENCH_TIMING
#include <cstring>
#include <ctime>
namespace PIN_IK {
struct WorkerTestAccess {
  static void hook(void (*fn)(PIN_IK&, const char*, unsigned)) { PIN_IK::test_hook = fn; }
};
}
using TraceClock = std::chrono::steady_clock;
struct Trace {
  TraceClock::time_point dispatch, start[2], finish[2], complete[2];
  double cpu_start[2]{}, cpu[2]{};
};
// One solver per process. Dispatch/completion barriers publish these fields;
// each worker writes only its own array elements, and the caller reads on return.
static Trace trace;
static double threadCPU() {
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value)) throw std::runtime_error("Thread CPU clock unavailable");
  return value.tv_sec*1e6 + value.tv_nsec*1e-3;
}
static void traceHook(PIN_IK::PIN_IK&, const char* event, unsigned index) {
  if (!std::strcmp(event,"before_dispatch")) trace.dispatch = TraceClock::now();
  else if (!std::strcmp(event,"before_solve")) {
    trace.start[index] = TraceClock::now(); trace.cpu_start[index] = threadCPU();
  } else if (!std::strcmp(event,"after_solve")) {
    trace.cpu[index] = threadCPU()-trace.cpu_start[index]; trace.finish[index] = TraceClock::now();
  } else if (!std::strcmp(event,"completed")) trace.complete[index] = TraceClock::now();
}
static double traceUS(TraceClock::time_point end, TraceClock::time_point begin) {
  return std::chrono::duration<double,std::micro>(end-begin).count();
}
#endif

using Pose = Eigen::Isometry3d;
using Joints = Eigen::Matrix<double, 6, 1>;
struct Joint {
  Eigen::Vector3d offset, axis;
  bool prismatic = false, continuous = false;
  double lower = -2.5, upper = 2.5;
};
struct Robot {
  std::array<Joint, 6> joints;
  Pose tool = Pose::Identity();
  Robot(const std::string& name) {
    if (name == "cartesian") {
      for (int i = 0; i < 6; ++i) {
        joints[i].offset.setZero();
        joints[i].axis = Eigen::Vector3d::Unit(i % 3);
        joints[i].prismatic = i < 3;
        joints[i].continuous = i == 5;
        joints[i].lower = -1; joints[i].upper = 1;
      }
    } else if (name == "arm6") {
      const double offsets[6][3] = {{0,0,.3}, {.1,0,.1}, {.3,0,0}, {.2,0,.1}, {.1,0,0}, {.08,0,0}};
      const int axes[6] = {2,1,1,0,1,2};
      for (int i = 0; i < 6; ++i) {
        joints[i].offset = Eigen::Vector3d(offsets[i][0], offsets[i][1], offsets[i][2]);
        joints[i].axis = Eigen::Vector3d::Unit(axes[i]);
      }
    } else throw std::invalid_argument("Unknown robot");
    tool.translation() = Eigen::Vector3d(.1,.02,.03);
    tool.linear() = (Eigen::AngleAxisd(.3, Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitX())).toRotationMatrix();
  }
  Pose fk(const Joints& q) const {
    Pose pose = Pose::Identity();
    for (int i = 0; i < 6; ++i) {
      pose.translate(joints[i].offset);
      if (joints[i].prismatic) pose.translate(joints[i].axis * q[i]);
      else pose.rotate(Eigen::AngleAxisd(q[i], joints[i].axis));
    }
    return pose * tool;
  }
  std::string xml() const {
    std::ostringstream s;
    s << "<robot name='comparison'><link name='base'/>";
    for (int i = 0; i < 6; ++i) {
      const auto& j = joints[i];
      s << "<link name='l" << i << "'/><joint name='j" << i << "' type='"
        << (j.prismatic ? "prismatic" : j.continuous ? "continuous" : "revolute")
        << "'><parent link='" << (i ? "l" + std::to_string(i-1) : "base")
        << "'/><child link='l" << i << "'/><origin xyz='" << j.offset.transpose()
        << "'/><axis xyz='" << j.axis.transpose() << "'/>";
      if (!j.continuous)
        s << "<limit lower='" << j.lower << "' upper='" << j.upper << "' effort='10' velocity='2'/>";
      s << "</joint>";
    }
    s << "<link name='tip'/><joint name='tool' type='fixed'><parent link='l5'/>"
         "<child link='tip'/><origin xyz='0.1 0.02 0.03' rpy='0.1 0.2 0.3'/></joint></robot>";
    return s.str();
  }
};

template<class T>
auto seedSolver(T& solver, int) -> decltype(solver.setRandomSeed(42), void()) { solver.setRandomSeed(42); }
template<class T> void seedSolver(T&, long) {}

struct Solver {
#ifdef BENCH_KDL
  TRAC_IK::TRAC_IK solver;
  KDL::JntArray seed{6}, out{6};
  KDL::Chain chain;
  Solver(const Robot& robot, double timeout, int mode)
      : solver("base", "tip", robot.xml(), timeout, 1e-6, static_cast<TRAC_IK::SolveType>(mode)) {
    solver.getKDLChain(chain);
  }
  Pose nativeFK(const Joints& q) {
    KDL::JntArray input(6); input.data = q;
    KDL::Frame frame;
    KDL::ChainFkSolverPos_recursive fk(chain);
    if (fk.JntToCart(input, frame) < 0) throw std::runtime_error("KDL FK failed");
    Pose pose = Pose::Identity();
    for (int r = 0; r < 3; ++r) {
      pose.translation()[r] = frame.p[r];
      for (int c = 0; c < 3; ++c) pose.linear()(r,c) = frame.M(r,c);
    }
    return pose;
  }
  int solve(const Joints& q, const Pose& pose, Joints& result, double& us) {
    seed.data = q;
    KDL::Frame target;
    for (int r = 0; r < 3; ++r) {
      target.p[r] = pose.translation()[r];
      for (int c = 0; c < 3; ++c) target.M(r,c) = pose.linear()(r,c);
    }
    const auto begin = std::chrono::steady_clock::now();
    const int rc = solver.CartToJnt(seed, target, out);
    us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-begin).count();
    result = out.data;
    return rc;
  }
#else
  PIN_IK::PIN_IK solver;
  Eigen::VectorXd seed{6}, out{6};
  pinocchio::Model model;
  Solver(const Robot& robot, double timeout, int mode)
      : solver("base", "tip", robot.xml(), timeout, 1e-6, static_cast<PIN_IK::SolveType>(mode)) {
    solver.getModel(model);
  }
  Pose nativeFK(const Joints& q) {
    pinocchio::Data data(model);
    pinocchio::forwardKinematics(model, data, PIN_IK::toPinocchioConfiguration(model, q));
    pinocchio::updateFramePlacements(model, data);
    const auto& frame = data.oMf[model.getFrameId("tip", pinocchio::BODY)];
    Pose pose = Pose::Identity(); pose.linear() = frame.rotation(); pose.translation() = frame.translation();
    return pose;
  }
  int solve(const Joints& q, const Pose& pose, Joints& result, double& us) {
    seed = q;
    pinocchio::SE3 target(pose.linear(), pose.translation());
    const auto begin = std::chrono::steady_clock::now();
    const int rc = solver.CartToJnt(seed, target, out);
    us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now()-begin).count();
    result = out;
    return rc;
  }
#endif
};

int main(int argc, char** argv) {
  try {
    if (argc != 8) throw std::invalid_argument(
        "Usage: ik_benchmark output.csv samples timeout_seconds mode scenario robot period_us");
    const int samples = std::stoi(argv[2]), mode = std::stoi(argv[4]);
    const double timeout = std::stod(argv[3]);
    const std::string scenario = argv[5];
    const int period_us = std::stoi(argv[7]);
    if (samples <= 0 || mode < 0 || mode > 4 || period_us < 0) throw std::invalid_argument("Invalid arguments");
    if (scenario != "smooth" && scenario != "exact" && scenario != "jump" &&
        scenario != "limits" && scenario != "singular" && scenario != "unreachable")
      throw std::invalid_argument("Unknown scenario");
    Robot robot(argv[6]);
    Solver solver(robot, timeout, mode);
#ifdef BENCH_TIMING
    PIN_IK::WorkerTestAccess::hook(traceHook);
#endif
    std::srand(42);
    seedSolver(solver.solver, 0);
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> random(-.7,.7);
    struct Case { Joints seed; Pose target; };
    std::vector<Case> cases;
    cases.reserve(samples);
    for (int i = 0; i < samples; ++i) {
      Joints q;
      for (int j = 0; j < 6; ++j) {
        q[j] = .4 * std::sin(i * .003 + j * .6);
        if (scenario == "jump") q[j] = random(rng);
        if (scenario == "limits") q[j] = (j % 2 ? -1 : 1) * (robot.joints[j].upper - .03);
        if (scenario == "singular") {
          // This 6R geometry loses rank at shoulder=-pi/2, other angles=0.
          q[j] = (j == 1 ? -M_PI/2 : 0) + 1e-3 * std::sin(i * .1 + j);
        }
        if (robot.joints[j].continuous) q[j] += 8 * M_PI;
      }
      Case test{q, robot.fk(q)};
      if (scenario != "exact") {
        for (int j = 0; j < 6; ++j) test.seed[j] += (j % 2 ? -.01 : .01);
      }
      if (scenario == "unreachable") test.target.translation().setConstant(20);
      if (i < 100 && !solver.nativeFK(q).matrix().isApprox(robot.fk(q).matrix(), 1e-11))
        throw std::runtime_error("Native FK does not match independent analytic model");
      cases.push_back(test);
    }
    Joints output;
    double elapsed;
    for (int i = 0; i < 100; ++i) solver.solve(cases[i % samples].seed, cases[i % samples].target, output, elapsed);
    struct Row {
      int rc; double us, position, rotation, limit, lateness; Joints q;
#ifdef BENCH_TIMING
      Trace timings;
      double return_wait;
#endif
    };
    std::vector<Row> rows(samples);
    auto release = std::chrono::steady_clock::now();
    for (int i = 0; i < samples; ++i) {
      if (period_us) { release += std::chrono::microseconds(period_us); std::this_thread::sleep_until(release); }
      auto& row = rows[i];
      row.rc = solver.solve(cases[i].seed, cases[i].target, row.q, row.us);
#ifdef BENCH_TIMING
      row.return_wait = traceUS(TraceClock::now(), std::max(trace.complete[0],trace.complete[1]));
      row.timings = trace;
#endif
      row.lateness = period_us ? std::chrono::duration<double, std::micro>(
          std::chrono::steady_clock::now() - release).count() : 0;
    }
    // Validation and file output are outside all timed calls and periodic release loops.
    std::ofstream csv(argv[1]);
    if (!csv) throw std::runtime_error("Cannot open output CSV");
    csv << "case,rc,us,position_error,rotation_error,limit_violation,release_to_finish_us";
    for (int j = 0; j < 6; ++j) csv << ",seed" << j;
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) csv << ",target" << r << c;
    for (int j = 0; j < 6; ++j) csv << ",q" << j;
#ifdef BENCH_TIMING
    for (int j = 0; j < 2; ++j)
      csv << ",dispatch_to_start" << j << ",worker_wall" << j << ",worker_cpu" << j;
    csv << ",first_finish_to_complete,last_complete_to_sample";
#endif
    csv << '\n' << std::setprecision(17);
    int success = 0, invalid = 0, over_ms = 0;
    double sum = 0, maximum = 0, max_position = 0, max_rotation = 0;
    for (int i = 0; i < samples; ++i) {
      auto& row = rows[i];
      const Pose actual = robot.fk(row.q);
      row.position = (actual.translation() - cases[i].target.translation()).norm();
      row.rotation = Eigen::AngleAxisd(actual.linear().transpose() * cases[i].target.linear()).angle();
      row.limit = 0;
      for (int j = 0; j < 6; ++j) if (!robot.joints[j].continuous)
        row.limit = std::max({row.limit, robot.joints[j].lower - row.q[j], row.q[j] - robot.joints[j].upper});
      if (row.rc >= 0) {
        ++success;
        if (!row.q.allFinite() || row.position > 5e-6 || row.rotation > 5e-6 || row.limit > 1e-8) ++invalid;
        max_position = std::max(max_position, row.position);
        max_rotation = std::max(max_rotation, row.rotation);
      } else if (!row.q.isApprox(cases[i].seed, 1e-12)) ++invalid;
      sum += row.us; maximum = std::max(maximum, row.us); over_ms += row.us > 1000;
      csv << i << ',' << row.rc << ',' << row.us << ',' << row.position << ',' << row.rotation
          << ',' << row.limit << ',' << row.lateness;
      for (int j = 0; j < 6; ++j) csv << ',' << cases[i].seed[j];
      for (int r = 0; r < 3; ++r) for (int c = 0; c < 4; ++c) csv << ',' << cases[i].target.matrix()(r,c);
      for (int j = 0; j < 6; ++j) csv << ',' << row.q[j];
#ifdef BENCH_TIMING
      const auto& t = row.timings;
      for (int j = 0; j < 2; ++j)
        csv << ',' << traceUS(t.start[j],t.dispatch) << ',' << traceUS(t.finish[j],t.start[j]) << ',' << t.cpu[j];
      csv << ',' << traceUS(std::max(t.complete[0],t.complete[1]),std::min(t.finish[0],t.finish[1]))
          << ',' << row.return_wait;
#endif
      csv << '\n';
    }
    std::cout << "samples=" << samples << " success=" << success << " invalid=" << invalid
              << " mean_us=" << sum/samples << " max_us=" << maximum << " over_1ms=" << over_ms
              << " max_position_error=" << max_position << " max_rotation_error=" << max_rotation << '\n';
    if (invalid || (scenario == "unreachable" && success)) return 1;
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
