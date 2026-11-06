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

#include <pin_ik/pinocchio_tl.hpp>
#include <pin_ik/solve_deadline.hpp>
#include <cfloat>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

namespace PIN_IK
{

ChainIkSolverPos_TL::ChainIkSolverPos_TL(const pinocchio::Model& _model,
                                          const Eigen::VectorXd& _q_min,
                                          const Eigen::VectorXd& _q_max,
                                          pinocchio::FrameIndex _tip_frame_id,
                                          double _maxtime, double _eps,
                                          bool _random_restart, bool _try_jl_wrap):
  model(_model), tip_frame_id(_tip_frame_id), q_min(_q_min), q_max(_q_max),
  delta_q(_model.nv), configuration(_model.nq), q_curr(_model.nv), diff_q(_model.nv),
  jacobian(6, _model.nv), decomposition(6, _model.nv),
  maxtime(_maxtime), eps(_eps), rr(_random_restart), wrap(_try_jl_wrap)
{
  ::PIN_IK::validateLimits(model, _q_min, _q_max);
  if (tip_frame_id >= model.frames.size()) throw std::invalid_argument("Invalid tip frame");

  data.reset(new pinocchio::Data(model));
  reset();

  types = ::PIN_IK::jointTypes(model);
  tip_covers_model = model.supports[model.frames[tip_frame_id].parentJoint].size() == model.njoints;

}

#ifdef PIN_IK_TESTING
void (*ChainIkSolverPos_TL::test_hook)(ChainIkSolverPos_TL&, const char*) = nullptr;
#endif

int ChainIkSolverPos_TL::CartToJnt(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                                    Eigen::VectorXd &q_out, const pinocchio::Motion _bounds)
{
  return CartToJntUntil(q_init, p_in, q_out, _bounds,
                       detail::deadlineAfter(std::chrono::steady_clock::now(), maxtime));
}

int ChainIkSolverPos_TL::CartToJntUntil(const Eigen::VectorXd &q_init, const pinocchio::SE3 &p_in,
                                     Eigen::VectorXd &q_out, const pinocchio::Motion& _bounds,
                                     std::chrono::steady_clock::time_point deadline)
{
  if (q_init.size() != model.nv || !q_init.allFinite()) return -1;
  if (aborted)
    return -3;

  q_out = q_init;
  bounds = _bounds;
  const auto stopped = [&] { return aborted || std::chrono::steady_clock::now() >= deadline; };
  while (!stopped())
  {
#ifdef PIN_IK_TESTING
    if (test_hook) test_hook(*this, "before_fk");
#endif
    if (stopped()) break;
    // Forward kinematics
    ::PIN_IK::toPinocchioConfiguration(model, q_out, configuration);
    pinocchio::forwardKinematics(model, *data, configuration);
    f = pinocchio::updateFramePlacement(model, *data, tip_frame_id);
    if (stopped()) break;

    delta_twist = diffRelative(p_in, f);

    // Apply bounds
    if (std::abs(delta_twist.linear()[0]) <= std::abs(bounds.linear()[0]))
      delta_twist.linear()[0] = 0;
    if (std::abs(delta_twist.linear()[1]) <= std::abs(bounds.linear()[1]))
      delta_twist.linear()[1] = 0;
    if (std::abs(delta_twist.linear()[2]) <= std::abs(bounds.linear()[2]))
      delta_twist.linear()[2] = 0;
    if (std::abs(delta_twist.angular()[0]) <= std::abs(bounds.angular()[0]))
      delta_twist.angular()[0] = 0;
    if (std::abs(delta_twist.angular()[1]) <= std::abs(bounds.angular()[1]))
      delta_twist.angular()[1] = 0;
    if (std::abs(delta_twist.angular()[2]) <= std::abs(bounds.angular()[2]))
      delta_twist.angular()[2] = 0;

    if (isMotionZero(delta_twist, eps))
      return 1;

    // log6(current^-1 * target) is expressed in the current tip's local frame.
    // Use a LOCAL Jacobian so the error and Jacobian share the same axes.
    pinocchio::SE3 diff = f.actInv(p_in);
    delta_twist = pinocchio::log6(diff);

    // Compute Jacobian
    if (stopped()) break;
    jacobian.setZero();
    if (tip_covers_model) {
      // FK above already populated joint placements/subspaces. Reuse them without
      // another forward pass; retain the tip-only path for models with branches.
      pinocchio::computeJointJacobians(model, *data);
      pinocchio::getFrameJacobian(model, *data, tip_frame_id, pinocchio::LOCAL, jacobian);
    } else {
      pinocchio::computeFrameJacobian(model, *data, configuration, tip_frame_id,
                                       pinocchio::LOCAL, jacobian);
    }
    if (stopped()) break;

    // Solve for delta_q using pseudo-inverse
    decomposition.compute(jacobian);
    if (stopped()) break;
    delta_q = decomposition.solve(delta_twist.toVector());
    if (stopped()) break;

    // Scalar angles are the coordinates of the supported one-DOF joints.
    q_curr = q_out + delta_q;

    // Apply joint limits
    for (int j = 0; j < q_min.size(); j++)
    {
      if (types[j] == Continuous)
        continue;

      if (q_curr(j) < q_min(j))
      {
        if (!wrap || types[j] == TransJoint)
          q_curr(j) = q_min(j);
        else
        {
          double diffangle = fmod(q_min(j) - q_curr(j), 2 * M_PI);
          double curr_angle = q_min(j) - diffangle + 2 * M_PI;
          if (curr_angle > q_max(j))
            q_curr(j) = q_min(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    for (int j = 0; j < q_max.size(); j++)
    {
      if (types[j] == Continuous)
        continue;

      if (q_curr(j) > q_max(j))
      {
        if (!wrap || types[j] == TransJoint)
          q_curr(j) = q_max(j);
        else
        {
          double diffangle = fmod(q_curr(j) - q_max(j), 2 * M_PI);
          double curr_angle = q_max(j) + diffangle - 2 * M_PI;
          if (curr_angle < q_min(j))
            q_curr(j) = q_max(j);
          else
            q_curr(j) = curr_angle;
        }
      }
    }

    diff_q = q_out - q_curr;

    if (diff_q.isZero(FLT_EPSILON))
    {
      if (rr)
      {
        if (stopped()) break;
        for (int j = 0; j < q_out.size(); j++)
          if (types[j] == Continuous)
            q_curr(j) = fRand(q_curr(j) - 2 * M_PI, q_curr(j) + 2 * M_PI);
          else
            q_curr(j) = fRand(q_min(j), q_max(j));
      }
    }

    q_out = q_curr;

  }

  return -3;
}

ChainIkSolverPos_TL::~ChainIkSolverPos_TL()
{
}

}
