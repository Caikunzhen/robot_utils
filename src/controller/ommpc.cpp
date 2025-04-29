/**
 *******************************************************************************
 * @file      : ommpc.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */

#ifdef HAS_QPOASES
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/controller/ommpc.hpp"

#include "robot_utils/geometry/core.hpp"
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void OmmpcParams::LoadParamsFromYamlNode(const YAML::Node& node,
                                         OmmpcParams& params)
{
  params.n = node["n"].as<size_t>();
  params.m = node["m"].as<size_t>();
  params.horizon = node["horizon"].as<size_t>();
  params.max_iter = node["max_iter"].as<qpOASES::int_t>();
  params.dt = node["dt"].as<real_t>();

  params.Q.resize(params.n);
  std::vector<real_t> Q_diag;
  node["Q_diag"].as<std::vector<real_t>>(Q_diag);
  PARAM_ASSERT(
      Q_diag.size() == params.n,
      "Q matrix size is not correct, expected size: %d, actual size: %d",
      params.n, Q_diag.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.Q.diagonal()[i] = Q_diag[i];
  }

  params.R.resize(params.m);
  std::vector<real_t> R_diag;
  node["R_diag"].as<std::vector<real_t>>(R_diag);
  PARAM_ASSERT(
      R_diag.size() == params.m,
      "R matrix size is not correct, expected size: %d, actual size: %d",
      params.m, R_diag.size());
  for (size_t i = 0; i < params.m; ++i) {
    params.R.diagonal()[i] = R_diag[i];
  }

  params.P.resize(params.n);
  std::vector<real_t> P_diag;
  node["P_diag"].as<std::vector<real_t>>(P_diag);
  PARAM_ASSERT(
      P_diag.size() == params.n,
      "P matrix size is not correct, expected size: %d, actual size: %d",
      params.n, P_diag.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.P.diagonal()[i] = P_diag[i];
  }

  params.u_min_.resize(params.m);
  std::vector<real_t> u_min;
  node["u_min"].as<std::vector<real_t>>(u_min);
  PARAM_ASSERT(u_min.size() == params.m,
               "u_min size is not correct, expected size: %d, actual size: %d",
               params.m, u_min.size());
  for (size_t i = 0; i < params.m; ++i) {
    params.u_min_(i) = u_min[i];
  }

  params.u_max_.resize(params.m);
  std::vector<real_t> u_max;
  node["u_max"].as<std::vector<real_t>>(u_max);
  PARAM_ASSERT(u_max.size() == params.m,
               "u_max size is not correct, expected size: %d, actual size: %d",
               params.m, u_max.size());
  for (size_t i = 0; i < params.m; ++i) {
    params.u_max_(i) = u_max[i];
  }
}

Ommpc::Ommpc(const Params& params)
{
  setParams(params);

  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  qp_.setOptions(op);
}

bool Ommpc::solve(const StateSeq& x_ref_seq, const CtrlSeq& u_ref_seq,
                  const StateVec& x0)
{
  PARAM_ASSERT(x_ref_seq.size() == params_.horizon,
               "x_ref_seq size must be equal to horizon");
  PARAM_ASSERT(u_ref_seq.size() == params_.horizon,
               "u_ref_seq size must be equal to horizon");
  PARAM_ASSERT(x0.dim() == params_.n, "x0 size must be equal to n");

  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  const size_t& m = params_.m;

  x_ref_seq_ = x_ref_seq;
  u_ref_seq_ = u_ref_seq;

  Eigen::MatrixX<real_t> F_x_i, F_u_i;
  Eigen::MatrixX<real_t> F_x_mul = Eigen::MatrixX<real_t>::Identity(n, n);
  for (size_t i = 0; i < N; ++i) {
    PARAM_ASSERT(
        x_ref_seq[i].dim() == n,
        "x_ref_seq[%zu] size must be equal to n, expected %zu, got %zu", i, n,
        x_ref_seq[i].dim());
    PARAM_ASSERT(
        u_ref_seq[i].size() == m,
        "u_ref_seq[%zu] size must be equal to m, expected %zu, got %zu", i, m,
        u_ref_seq[i].size());

    getJacobian(x_ref_seq[i], u_ref_seq[i], F_x_i, F_u_i);
    for (size_t j = 0; j < i; ++j) {
      S_bar_.block(i * n, j * m, n, m) =
          F_x_i * S_bar_.block((i - 1) * n, j * m, n, m);
    }
    S_bar_.block(i * n, i * m, n, m) = F_u_i;
    F_x_mul = F_x_i * F_x_mul;
    T_bar_.block(i * n, 0, n, n) = F_x_mul;
    lb_.segment(i * m, m) = params_.u_min_ - u_ref_seq[i];
    ub_.segment(i * m, m) = params_.u_max_ - u_ref_seq[i];
  }

  Eigen::MatrixX<real_t> tmp = S_bar_.transpose() * Q_bar_;
  H_ = tmp * S_bar_;
  H_.diagonal() += R_bar_.diagonal();
  dx0_ = x0 - x_ref_seq[0];
  g_ = tmp * T_bar_ * dx0_;

  qpOASES::returnValue ret;
  qpOASES::int_t nWSR = params_.max_iter;
  ret = qp_.init(H_.data(), g_.data(), nullptr, lb_.data(), ub_.data(), nullptr,
                 nullptr, nWSR);

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    solved_ = false;
  } else {
    qp_.getPrimalSolution(dU_bar_.data());
    solved_ = true;
  }

  return solved_;
}

void Ommpc::getCtrl(InputVec& u, size_t forward_steps) const
{
  if (forward_steps >= params_.horizon) {
    forward_steps = params_.horizon - 1;
  }

  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u = dU_bar_.segment(forward_steps * params_.m, params_.m) +
      u_ref_seq_[forward_steps];
}

void Ommpc::getCtrlSeq(CtrlSeq& u_seq) const
{
  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u_seq.clear();
  for (size_t i = 0; i < params_.horizon; ++i) {
    u_seq.emplace_back(dU_bar_.segment(i * params_.m, params_.m) +
                       u_ref_seq_[i]);
  }
}

void Ommpc::getPredStateSeq(StateSeq& x_seq) const
{
  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  Eigen::VectorX<real_t> dx_bar = S_bar_ * dU_bar_ + T_bar_ * dx0_;
  x_seq.clear();

  x_seq.emplace_back(x_ref_seq_[0] + dx0_);
  for (size_t i = 1; i < params_.horizon; ++i) {
    x_seq.emplace_back(x_ref_seq_[i] +
                       dx_bar.segment((i - 1) * params_.n, params_.n));
  }
}

void Ommpc::setParams(const Params& params)
{
  PARAM_ASSERT(params.n > 0, "The number of states must be greater than 0");
  PARAM_ASSERT(params.m > 0, "The number of inputs must be greater than 0");
  PARAM_ASSERT(params.horizon > 0,
               "The prediction horizon must be greater than 0");
  PARAM_ASSERT(params.max_iter > 0,
               "The maximum number of iterations must be greater than 0");
  PARAM_ASSERT(params.f, "f must be a non-null function");
  PARAM_ASSERT(params.f_dx, "f_dx must be a non-null function");
  PARAM_ASSERT(params.f_du, "f_du must be a non-null function");
  PARAM_ASSERT(params.Q.rows() == params.n && params.Q.cols() == params.n,
               "Q must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.R.rows() == params.m && params.R.cols() == params.m,
               "R must be a square matrix of size (m, m)");
  PARAM_ASSERT(params.P.rows() == params.n && params.P.cols() == params.n,
               "P must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.u_min_.size() == params.m,
               "u_min_ size must be equal to m");
  PARAM_ASSERT(params.u_max_.size() == params.m,
               "u_max_ size must be equal to m");

  params_ = params;

  S_bar_.resize(params_.n * params_.horizon, params_.m * params_.horizon);
  T_bar_.resize(params_.n * params_.horizon, params_.n);
  dU_bar_.resize(params_.horizon * params_.m);
  lb_.resize(params_.horizon * params_.m);
  ub_.resize(params_.horizon * params_.m);

  calcQBar();

  calcRBar();
}

void Ommpc::calcQBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  Q_bar_.resize(n * N);
  const auto& Q = params_.Q;
  const auto& P = params_.P;

  if (N == 1) {
    Q_bar_.diagonal() = P.diagonal();
  } else {
    Q_bar_.diagonal() << Q.diagonal().replicate(N - 1, 1), P.diagonal();
  }
}

void Ommpc::calcRBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& m = params_.m;
  R_bar_.resize(m * N);
  const auto& R = params_.R;
  R_bar_.diagonal() = R.diagonal().replicate(N, 1);
}

void Ommpc::getManifoldSpecificJacobian(
    const StateVec& x_ref, const ManifoldBase<real_t>::HomeSpace& delta,
    Eigen::MatrixX<real_t>& G_x, Eigen::MatrixX<real_t>& G_f)
{
  G_x = G_f = Eigen::MatrixX<real_t>::Zero(x_ref.dim(), x_ref.dim());
  size_t start_idx = 0;
  for (size_t i = 0; i < x_ref.data().size(); ++i) {
    const auto& prim_m = x_ref.data()[i];
    const auto& dim = prim_m->dim();
    Eigen::MatrixX<real_t> G_x_tmp, G_f_tmp;
    getPrimManifoldSpecificJacobian(*prim_m, delta.segment(start_idx, dim),
                                    G_x_tmp, G_f_tmp);
    G_x.block(start_idx, start_idx, dim, dim) = G_x_tmp;
    G_f.block(start_idx, start_idx, dim, dim) = G_f_tmp;
    start_idx += dim;
  }
}

void Ommpc::getPrimManifoldSpecificJacobian(
    const ManifoldBase<real_t>& prim_m,
    const ManifoldBase<real_t>::HomeSpace& delta, Eigen::MatrixX<real_t>& G_x,
    Eigen::MatrixX<real_t>& G_f)
{
  PARAM_ASSERT(prim_m.dim() == delta.size(),
               "Manifold dimension mismatch, expected %zu, got %zu",
               prim_m.dim(), delta.size());

  switch (prim_m.type()) {
    case ManifoldType::kEuclideanSpaceX:
    case ManifoldType::kSpecialOrthogonalGroup2:
    case ManifoldType::kSurface2D:
      G_x = G_f = Eigen::MatrixX<real_t>::Identity(prim_m.dim(), prim_m.dim());
      break;
    case ManifoldType::kSpecialOrthogonalGroup3: {
      ManifoldBase<real_t>::HomeSpace tmp = params_.dt * delta;
      Eigen::AngleAxis<real_t> angle_axis;
      angle_axis.angle() = tmp.norm();
      angle_axis.axis() = -tmp / angle_axis.angle();
      G_x = angle_axis.toRotationMatrix();
      Eigen::Matrix3<real_t> skew_tmp_norm = Vec2Skew(angle_axis.axis());
      real_t coef1 = (1 - cos(angle_axis.angle())) / angle_axis.angle();
      real_t coef2 = 1 - sin(angle_axis.angle()) / angle_axis.angle();
      G_f = (Eigen::Matrix3<real_t>::Identity() + coef1 * skew_tmp_norm +
             coef2 * skew_tmp_norm * skew_tmp_norm)
                .transpose();
    } break;
    default:
      PARAM_ASSERT(false, "Unsupported manifold type");
      break;
  }
}

void Ommpc::getJacobian(const StateVec& x_ref, const InputVec& u_ref,
                        Eigen::MatrixX<real_t>& F_x,
                        Eigen::MatrixX<real_t>& F_u)
{
  Eigen::MatrixX<real_t> G_x, G_f;
  Eigen::VectorX<real_t> delta = params_.f(x_ref, u_ref);
  getManifoldSpecificJacobian(x_ref, delta, G_x, G_f);
  F_x = G_x + params_.dt * G_f * params_.f_dx(x_ref, u_ref);
  F_u = params_.dt * G_f * params_.f_du(x_ref, u_ref);
}
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils

#endif /* HAS_QPOASES */