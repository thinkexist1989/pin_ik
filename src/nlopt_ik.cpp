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

#include <pin_ik/nlopt_ik.hpp>
#include <pin_ik/solve_deadline.hpp>
#include <limits>
#include <pin_ik/dual_quaternion.h>
#include <cmath>
#include <cfloat>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace NLOPT_IK
{

#ifdef PIN_IK_TESTING
void (*NLOPT_IK::test_hook)(NLOPT_IK&, const char*) = nullptr;
#endif

double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;
  return c->minJoints(x, grad);
}

double minfuncDQ(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;
  return c->objective(x, grad.empty() ? nullptr : grad.data(), &NLOPT_IK::cartDQError);
}

double minfuncSumSquared(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  return c->objective(x, grad.empty() ? nullptr : grad.data(), &NLOPT_IK::cartSumSquaredError);
}

double minfuncL2(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  return c->objective(x, grad.empty() ? nullptr : grad.data(), &NLOPT_IK::cartL2NormError);
}

void constrainfuncm(uint m, double* result, uint n, const double* x, double* grad, void* data)
{
  NLOPT_IK *c = (NLOPT_IK *) data;

  // Joint mode registers exactly one scalar equality constraint.
  if (m != 1 || n != c->x.size()) throw std::invalid_argument("Invalid constraint dimensions");
  std::copy(x, x + n, c->constraint_values.begin());
  result[0] = c->objective(c->constraint_values, grad, &NLOPT_IK::cartSumSquaredError);
}

bool NLOPT_IK::stopped() const
{
  return aborted || progress != -3 || std::chrono::steady_clock::now() >= deadline;
}

bool NLOPT_IK::stopCallback()
{
  if (!stopped()) return false;
  opt.force_stop(); // Only the optimizer's owning thread touches opt.
  return true;
}

double NLOPT_IK::objective(const std::vector<double>& values, double* grad, ErrorFunction error)
{
  if (grad) std::fill(grad, grad + values.size(), 0.0);
  double result = std::numeric_limits<float>::max();
  (this->*error)(values, &result);
  if (stopCallback() || !grad) return result;
  perturbation = values;
  const double jump = FLT_EPSILON;
  for (size_t i = 0; i < values.size(); ++i) {
    if (stopCallback()) break;
    perturbation[i] += jump;
    double perturbed = std::numeric_limits<float>::max();
    (this->*error)(perturbation, &perturbed);
    perturbation[i] = values[i];
    if (stopCallback()) break;
    // Keep the legacy gradient scale; correcting it is a separate stage C change.
    grad[i] = (perturbed - result) / (2.0 * jump);
  }
  return result;
}

NLOPT_IK::NLOPT_IK(const pinocchio::Model& _model, const Eigen::VectorXd& _q_min,
                   const Eigen::VectorXd& _q_max, pinocchio::FrameIndex _tip_frame_id,
                   double _maxtime, double _eps, OptType _type):
  model(_model), tip_frame_id(_tip_frame_id), maxtime(_maxtime), eps(std::abs(_eps)), TYPE(_type)
{
  PIN_IK::validateLimits(model, _q_min, _q_max);
  if (tip_frame_id >= model.frames.size()) throw std::invalid_argument("Invalid tip frame");

  data.reset(new pinocchio::Data(model));
  targetDQ = std::make_unique<dual_quaternion>();
  x.resize(model.nv);
  perturbation.resize(model.nv);
  constraint_values.resize(model.nv);
  artificial_lower_limits.resize(model.nv);
  artificial_upper_limits.resize(model.nv);
  des.resize(model.nv);
  best_x.resize(model.nv);
  angles.resize(model.nv);
  configuration.resize(model.nq);
  reset();

  opt = nlopt::opt(nlopt::LD_SLSQP, model.nv);

  for (int i = 0; i < model.nv; i++)
  {
    lb.push_back(_q_min(i));
    ub.push_back(_q_max(i));
  }

  types = PIN_IK::jointTypes(model);

  std::vector<double> tolerance(1, FLT_EPSILON);
  opt.set_xtol_abs(tolerance[0]);

  switch (TYPE)
  {
  case Joint:
    opt.set_min_objective(minfunc, this);
    opt.add_equality_mconstraint(constrainfuncm, this, tolerance);
    break;
  case DualQuat:
    opt.set_min_objective(minfuncDQ, this);
    break;
  case SumSq:
    opt.set_min_objective(minfuncSumSquared, this);
    break;
  case L2:
    opt.set_min_objective(minfuncL2, this);
    break;
  }
}

NLOPT_IK::~NLOPT_IK() = default;

double NLOPT_IK::minJoints(const std::vector<double>& x, std::vector<double>& grad)
{
  std::fill(grad.begin(), grad.end(), 0.0);
  if (stopCallback()) return std::numeric_limits<float>::max();
  bool gradient = !grad.empty();

  double err = 0;
  for (uint i = 0; i < x.size(); i++)
  {
    err += pow(x[i] - des[i], 2);
    if (gradient)
      grad[i] = 2.0 * (x[i] - des[i]);
  }

  return err;
}

void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
{
  if (x.size() != static_cast<size_t>(model.nv))
    throw std::invalid_argument("Invalid callback joint dimensions");
  error[0] = std::numeric_limits<float>::max();
#ifdef PIN_IK_TESTING
  if (test_hook) test_hook(*this, "error_callback");
#endif
  if (stopCallback()) return;
  for (uint i = 0; i < x.size(); i++)
    angles(i) = x[i];

  PIN_IK::toPinocchioConfiguration(model, angles, configuration);
  pinocchio::forwardKinematics(model, *data, configuration);
  currentPose = pinocchio::updateFramePlacement(model, *data, tip_frame_id);
  if (stopCallback()) return;

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  error[0] = delta_twist.linear().squaredNorm() + delta_twist.angular().squaredNorm();

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[])
{
  if (x.size() != static_cast<size_t>(model.nv))
    throw std::invalid_argument("Invalid callback joint dimensions");
  error[0] = std::numeric_limits<float>::max();
#ifdef PIN_IK_TESTING
  if (test_hook) test_hook(*this, "error_callback");
#endif
  if (stopCallback()) return;
  for (uint i = 0; i < x.size(); i++)
    angles(i) = x[i];

  PIN_IK::toPinocchioConfiguration(model, angles, configuration);
  pinocchio::forwardKinematics(model, *data, configuration);
  currentPose = pinocchio::updateFramePlacement(model, *data, tip_frame_id);
  if (stopCallback()) return;

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  error[0] = std::sqrt(delta_twist.linear().squaredNorm() + delta_twist.angular().squaredNorm());

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

void NLOPT_IK::cartDQError(const std::vector<double>& x, double error[])
{
  if (x.size() != static_cast<size_t>(model.nv))
    throw std::invalid_argument("Invalid callback joint dimensions");
  error[0] = std::numeric_limits<float>::max();
#ifdef PIN_IK_TESTING
  if (test_hook) test_hook(*this, "error_callback");
#endif
  if (stopCallback()) return;
  for (uint i = 0; i < x.size(); i++)
    angles(i) = x[i];

  PIN_IK::toPinocchioConfiguration(model, angles, configuration);
  pinocchio::forwardKinematics(model, *data, configuration);
  currentPose = pinocchio::updateFramePlacement(model, *data, tip_frame_id);
  if (stopCallback()) return;

  if (std::isnan(currentPose.translation().x()))
  {
    std::fprintf(stderr, "NaNs from NLOpt!!");
    error[0] = std::numeric_limits<float>::max();
    progress = -1;
    return;
  }

  pinocchio::Motion delta_twist = PIN_IK::diffRelative(targetPose, currentPose);

  for (int i = 0; i < 3; i++)
  {
    if (std::abs(delta_twist.linear()[i]) <= std::abs(bounds.linear()[i]))
      delta_twist.linear()[i] = 0.0;
    if (std::abs(delta_twist.angular()[i]) <= std::abs(bounds.angular()[i]))
      delta_twist.angular()[i] = 0.0;
  }

  // Convert SE3 to dual quaternion
  Eigen::Matrix3d R = currentPose.rotation();
  Eigen::Quaterniond currentQuaternion(R);
  Eigen::Vector3d currentTranslation = currentPose.translation();

  math3d::matrix3x3<double> currentRotationMatrix;
  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      currentRotationMatrix(i, j) = R(i,j);

  math3d::quaternion<double> currentQuat = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
  math3d::point3d currentTrans;
  currentTrans.x = currentTranslation.x();
  currentTrans.y = currentTranslation.y();
  currentTrans.z = currentTranslation.z();

  dual_quaternion currentDQ = dual_quaternion::rigid_transformation(currentQuat, currentTrans);

  dual_quaternion errorDQ = (currentDQ * !*targetDQ).normalize();
  errorDQ.log();
  error[0] = 4.0f * dot(errorDQ, errorDQ);

  if (PIN_IK::isMotionZero(delta_twist, eps))
  {
    progress = 1;
    best_x = x;
    return;
  }
}

int NLOPT_IK::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                        Eigen::VectorXd &q_out, const pinocchio::Motion _bounds,
                        const Eigen::VectorXd& q_desired)
{
  return CartToJntUntil(q_init, p_in, q_out, _bounds,
      PIN_IK::detail::deadlineAfter(std::chrono::steady_clock::now(), maxtime), q_desired);
}

int NLOPT_IK::CartToJntUntil(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                           Eigen::VectorXd &q_out, const pinocchio::Motion& _bounds,
                           std::chrono::steady_clock::time_point task_deadline,
                           const Eigen::VectorXd& q_desired)
{
  deadline = task_deadline;
  progress = -3;

  bounds = _bounds;
  q_out = q_init;

  if (q_init.size() != types.size() || !q_init.allFinite())
  {
    std::fprintf(stderr, "IK seeded with wrong number of joints.  Expected %d but got %d",
                 (int)types.size(), (int)q_init.size());
    return -3;
  }

  if (stopped()) return -3;
  if (q_desired.size() && (q_desired.size() != model.nv || !q_desired.allFinite()))
    throw std::invalid_argument("Invalid desired joint values");

  double minf;
  targetPose = p_in;

  if (TYPE == 1)   // DQ
  {
    Eigen::Matrix3d R = targetPose.rotation();
    math3d::matrix3x3<double> targetRotationMatrix;
    for(int i = 0; i < 3; i++)
      for(int j = 0; j < 3; j++)
        targetRotationMatrix(i, j) = R(i,j);

    math3d::quaternion<double> targetQuaternion = math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
    math3d::point3d targetTranslation;
    targetTranslation.x = targetPose.translation().x();
    targetTranslation.y = targetPose.translation().y();
    targetTranslation.z = targetPose.translation().z();
    *targetDQ = dual_quaternion::rigid_transformation(targetQuaternion, targetTranslation);
  }

  for (int i = 0; i < x.size(); i++)
  {
    x[i] = q_init(i);

    if (types[i] == PIN_IK::Continuous)
      continue;

    if (types[i] == PIN_IK::TransJoint)
    {
      x[i] = std::min(x[i], ub[i]);
      x[i] = std::max(x[i], lb[i]);
    }
    else
    {
      if (x[i] > ub[i])
      {
        double diffangle = fmod(x[i] - ub[i], 2 * M_PI);
        x[i] = ub[i] + diffangle - 2 * M_PI;
      }

      if (x[i] < lb[i])
      {
        double diffangle = fmod(lb[i] - x[i], 2 * M_PI);
        x[i] = lb[i] - diffangle + 2 * M_PI;
      }

      if (x[i] > ub[i])
        x[i] = (ub[i] + lb[i]) / 2.0;
    }
  }

  best_x = x;
  progress = -3;

  for (uint i = 0; i < lb.size(); i++)
    if (types[i] == PIN_IK::Continuous)
      artificial_lower_limits[i] = best_x[i] - 2 * M_PI;
    else if (types[i] == PIN_IK::TransJoint)
      artificial_lower_limits[i] = lb[i];
    else
      artificial_lower_limits[i] = std::max(lb[i], best_x[i] - 2 * M_PI);

  opt.set_lower_bounds(artificial_lower_limits);

  for (uint i = 0; i < ub.size(); i++)
    if (types[i] == PIN_IK::Continuous)
      artificial_upper_limits[i] = best_x[i] + 2 * M_PI;
    else if (types[i] == PIN_IK::TransJoint)
      artificial_upper_limits[i] = ub[i];
    else
      artificial_upper_limits[i] = std::min(ub[i], best_x[i] + 2 * M_PI);

  opt.set_upper_bounds(artificial_upper_limits);

  if (q_desired.size() == 0)
  {
    des = x;
  }
  else
  {
    for (uint i = 0; i < des.size(); i++)
      des[i] = q_desired(i);
  }

  while (!stopped())
  {
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "before_optimize");
#endif
    const double time_left = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    // NLopt interprets a non-positive maxtime as NO time limit.
    if (aborted || time_left <= 0) break;
    opt.set_maxtime(time_left);
    try
    {
      opt.optimize(x, minf);
    }
    catch (const nlopt::forced_stop&) {
      if (!stopped()) throw;
    }
    catch (const nlopt::roundoff_limited&) {
      // Expected numerical stagnation: retry within the same absolute budget.
    }
    if (progress == -1) progress = -3;
    if (stopped()) break;
    for (uint i = 0; i < x.size(); i++)
      x[i] = fRand(artificial_lower_limits[i], artificial_upper_limits[i]);
  }

  for (uint i = 0; i < x.size(); i++)
  {
    q_out(i) = best_x[i];
  }

  return progress;
}

}
