/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#include <pin_ik/pin_ik.hpp>
#include <pin_ik/solve_deadline.hpp>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <cstdio>
#include <iostream>
#include <limits>
#include <pin_ik/urdf.hpp>
#include <stdexcept>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace PIN_IK
{

PIN_IK::PIN_IK(const std::string& base, const std::string& tip, const std::string& xml,
                 double timeout, double tolerance, SolveType type) :
  initialized(false), eps(tolerance), maxtime(timeout), solvetype(type)
{
  loadURDFModel(xml, base, tip, model, lb, ub, tip_frame_id);
  initialize();
  startWorkers();
}

PIN_IK::PIN_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
                 const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
                 double _maxtime, double _eps, SolveType _type):
  initialized(false),
  model(_model),
  tip_frame_id(_tip_frame_id),
  lb(_q_min),
  ub(_q_max),
  eps(_eps),
  maxtime(_maxtime),
  solvetype(_type)
{
  initialize();
  startWorkers();
}

void PIN_IK::initialize()
{
  if (!std::isfinite(maxtime) || maxtime <= 0 || !std::isfinite(eps) || eps <= 0)
    throw std::invalid_argument("Timeout and epsilon must be finite and positive");

  if (model.nv == 0 || model.nv != lb.size() || lb.size() != ub.size())
    throw std::invalid_argument("Invalid model or joint limit dimensions");

  validateLimits(model, lb, ub);
  if (tip_frame_id >= model.frames.size())
    throw std::invalid_argument("Invalid tip frame");

  resetSolvers();
  for (auto& workspace : scoring) workspace = std::make_unique<ScoreWorkspace>(model);

  types = ::PIN_IK::jointTypes(model);

  initialized = true;
}

bool PIN_IK::setLimits(Eigen::VectorXd& lb_, Eigen::VectorXd& ub_)
{
  validateLimits(model, lb_, ub_);
  // Prepare everything before committing; allocation failures preserve the old limits.
  Eigen::VectorXd new_lb = lb_, new_ub = ub_;
  auto new_nl = std::make_unique<NLOPT_IK::NLOPT_IK>(
      model, new_lb, new_ub, tip_frame_id, maxtime, eps, NLOPT_IK::SumSq);
  auto new_ik = std::make_unique<ChainIkSolverPos_TL>(
      model, new_lb, new_ub, tip_frame_id, maxtime, eps, true, true);
  new_nl->random = nl_solver->random;
  new_ik->random = iksolver->random;
  std::lock_guard<std::mutex> lock(worker_mutex);
  lb.swap(new_lb);
  ub.swap(new_ub);
  nl_solver.swap(new_nl);
  iksolver.swap(new_ik);
  return true;
}

void PIN_IK::setRandomSeed(std::uint64_t seed)
{
  // Distinct deterministic streams; no process-global state or hot-path entropy.
  random[0].seed(seed);
  random[1].seed(seed ^ UINT64_C(0x9e3779b97f4a7c15));
  iksolver->setRandomSeed(seed ^ UINT64_C(0xbf58476d1ce4e5b9));
  nl_solver->setRandomSeed(seed ^ UINT64_C(0x94d049bb133111eb));
}

#ifdef PIN_IK_TESTING
void (*PIN_IK::test_hook)(PIN_IK&, const char*, unsigned) = nullptr;
#endif

void PIN_IK::startWorkers()
{
  task_seed.resize(model.nv);
  for (unsigned i = 0; i < 2; ++i) {
    worker_seed[i].resize(model.nv);
    worker_output[i].resize(model.nv);
    speed_slots[i].resize(model.nv);
  }
  solutions.reserve(2);
  errors.reserve(2);
  try
  {
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "before_start", 0);
#endif
    task1 = std::thread(&PIN_IK::workerLoop, this, 0);
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "before_start", 1);
#endif
    task2 = std::thread(&PIN_IK::workerLoop, this, 1);
    std::unique_lock<std::mutex> lock(worker_mutex);
    state_cv.wait(lock, [this] { return ready_workers == 2; });
  }
  catch (...)
  {
    // A failed constructor will not invoke ~PIN_IK().
    stopWorkers();
    throw;
  }
}

void PIN_IK::stopWorkers()
{
  {
    std::lock_guard<std::mutex> lock(worker_mutex);
    shutdown = true;
  }
  work_cv.notify_all();
  if (task1.joinable()) task1.join();
  if (task2.joinable()) task2.join();
}

void PIN_IK::workerLoop(unsigned index)
{
  std::uint64_t seen_generation = 0;
  std::unique_lock<std::mutex> lock(worker_mutex);
  ++ready_workers;
  state_cv.notify_one();
  while (true)
  {
    work_cv.wait(lock, [this, &seen_generation] {
      return shutdown || generation != seen_generation;
    });
    if (shutdown) return;
    seen_generation = generation;
    lock.unlock();

    std::exception_ptr error;
    try
    {
#ifdef PIN_IK_TESTING
      if (test_hook) test_hook(*this, "before_solve", index);
#endif
      if (index == 0) runPinocchioIK(task_seed, task_pose);
      else runNLOPT(task_seed, task_pose);
#ifdef PIN_IK_TESTING
      if (test_hook) test_hook(*this, "after_solve", index);
#endif
    }
    catch (...)
    {
      error = std::current_exception();
      iksolver->abort();
      nl_solver->abort();
    }
    lock.lock();
    worker_errors[index] = error;
    ++completed_workers;
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "completed", index);
#endif
    state_cv.notify_one();
    // No task/solver access after completion until the next generation is acquired.
  }
}

bool PIN_IK::unique_solution(const Eigen::VectorXd& sol)
{
  for (uint i = 0; i < solutions.size(); i++)
    if (myEqual(sol, solutions[i]))
      return false;
  return true;
}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    double diffangle = fmod(val - max, 2 * M_PI);
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    double diffangle = fmod(min - val, 2 * M_PI);
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  normalizeAngle(val, target - M_PI, target + M_PI);
}

template<typename T1, typename T2>
bool PIN_IK::runSolver(T1& solver, T2& other_solver,
                        const Eigen::VectorXd &q_init,
                        const pinocchio::SE3 &p_in, unsigned index)
{
  auto& q_out = worker_output[index];
  auto& seed = worker_seed[index];
  seed = q_init;

  while (!solver.aborted)
  {
    if (std::chrono::steady_clock::now() >= deadline)
      break;
    int RC = solver.CartToJntUntil(seed, p_in, q_out, bounds, deadline);
    if (RC >= 0)
    {
      switch (solvetype)
      {
      case Manip1:
      case Manip2:
      case Manip3:
        normalize_limits(q_init, q_out);
        break;
      default:
        normalize_seed(q_init, q_out);
        break;
      }
#ifdef PIN_IK_TESTING
      if (test_hook) test_hook(*this, "before_score", index);
#endif
      const double err = scoreSolution(q_init, q_out, index);
#ifdef PIN_IK_TESTING
      if (test_hook) test_hook(*this, "after_score", index);
#endif
      std::lock_guard<std::mutex> lock(mtx_);
#ifdef PIN_IK_TESTING
      if (test_hook) test_hook(*this, "before_result", index);
#endif
      if (unique_solution(q_out))
      {
        if (solvetype == Speed) {
          auto& slot = speed_slots[solutions.size()];
          slot = q_out;
          solutions.emplace_back();
          solutions.back().swap(slot);
        } else {
          solutions.push_back(q_out);
        }
        uint curr_size = solutions.size();
        errors.resize(curr_size);
        errors[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
    }

    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!solutions.empty() && solvetype == Speed) break;
    }

    if (solver.aborted || std::chrono::steady_clock::now() >= deadline) break;
    for (int j = 0; j < seed.size(); j++)
      if (types[j] == Continuous)
        seed(j) = random[index].uniform(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = random[index].uniform(lb(j), ub(j));
  }
#ifdef PIN_IK_TESTING
  if (test_hook) test_hook(*this, "before_cancel", index);
#endif
  other_solver.abort();

  return true;
}

void PIN_IK::normalize_seed(const Eigen::VectorXd& seed, Eigen::VectorXd& solution)
{
  for (int i = 0; i < lb.size(); i++)
  {
    if (types[i] == TransJoint)
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }
}

void PIN_IK::normalize_limits(const Eigen::VectorXd& seed, Eigen::VectorXd& solution)
{
  for (int i = 0; i < lb.size(); i++)
  {
    if (types[i] == TransJoint)
      continue;

    double target = seed(i);

    if (types[i] == RotJoint && types[i] != Continuous)
      target = (ub(i) + lb(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types[i] == Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, lb(i), ub(i));

    solution(i) = val;
  }
}

double PIN_IK::manipPenalty(const Eigen::VectorXd& arr)
{
  double penalty = 1.0;
  for (int i = 0; i < arr.size(); i++)
  {
    if (types[i] == Continuous)
      continue;
    double range = ub(i) - lb(i);
    penalty *= ((arr(i) - lb(i)) * (ub(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}

double PIN_IK::scoreSolution(const Eigen::VectorXd& seed, const Eigen::VectorXd& arr, unsigned index)
{
  if (solvetype != Manip1 && solvetype != Manip2 && solvetype != Manip3)
    return JointErr(seed, arr);
  auto& workspace = *scoring[index];
  toPinocchioConfiguration(model, arr, workspace.configuration);
  workspace.jacobian.setZero();
  pinocchio::forwardKinematics(model, workspace.data, workspace.configuration);
  pinocchio::computeFrameJacobian(model, workspace.data, workspace.configuration, tip_frame_id,
                                  pinocchio::LOCAL_WORLD_ALIGNED, workspace.jacobian);
  workspace.svd.compute(workspace.jacobian);
  const auto& singular_values = workspace.svd.singularValues();
  double value = singular_values.minCoeff();
  if (solvetype == Manip1) value = singular_values.prod();
  else if (solvetype == Manip2) value /= singular_values.maxCoeff();
  return manipPenalty(arr) * value;
}

int PIN_IK::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                       Eigen::VectorXd &q_out, const pinocchio::Motion& _bounds)
{
  const auto entry_time = std::chrono::steady_clock::now();
  if (q_init.size() != model.nv || !q_init.allFinite()) return -1;

  if (!initialized)
  {
    std::fprintf(stderr, "PIN-IK was not properly initialized with a valid model or limits.  IK cannot proceed");
    return -1;
  }

  deadline = detail::deadlineAfter(entry_time, maxtime);

  nl_solver->reset();
  iksolver->reset();

  // Recover the two Speed slots before clear() destroys their dynamic storage.
  for (unsigned i = 0; i < 2 && i < solutions.size(); ++i)
    if (speed_slots[i].size() == 0) speed_slots[i].swap(solutions[i]);
  solutions.clear();
  errors.clear();

  {
    std::unique_lock<std::mutex> lock(worker_mutex);
    bounds = _bounds;
    task_seed = q_init;
    task_pose = p_in;
    worker_errors = {};
    completed_workers = 0;
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "before_dispatch", 0);
#endif
    ++generation;
    lock.unlock();
    work_cv.notify_all();
    lock.lock();
    state_cv.wait(lock, [this] { return completed_workers == 2; });
  }
  for (const auto& error : worker_errors)
    if (error) std::rethrow_exception(error);

  if (solutions.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (solvetype)
  {
  case Manip1:
  case Manip2:
  case Manip3:
    std::sort(errors.rbegin(), errors.rend());
    break;
  default:
    std::sort(errors.begin(), errors.end());
    break;
  }

  q_out = solutions[errors[0].second];

  return solutions.size();
}

PIN_IK::~PIN_IK()
{
  // The owner must finish any public call before destroying this instance.
  stopWorkers();
}

}
