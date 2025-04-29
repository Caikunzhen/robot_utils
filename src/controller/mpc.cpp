/**
 *******************************************************************************
 * @file      : mpc.cpp
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
#include "robot_utils/controller/mpc.hpp"
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

void MpcParams::LoadParamsFromYamlNode(const YAML::Node& node,
                                       MpcParams& params)
{
  params.n = node["n"].as<size_t>();
  params.m = node["m"].as<size_t>();
  params.horizon = node["horizon"].as<size_t>();
  params.max_iter = node["max_iter"].as<qpOASES::int_t>();

  if (node["A"]) {
    params.A.resize(params.n, params.n);
    std::vector<real_t> A_flat;
    node["A"].as<std::vector<real_t>>(A_flat);
    PARAM_ASSERT(
        A_flat.size() == params.n * params.n,
        "A matrix size is not correct, expected size: %d, actual size: %d",
        params.n * params.n, A_flat.size());
    for (size_t i = 0; i < params.n; ++i) {
      for (size_t j = 0; j < params.n; ++j) {
        params.A(i, j) = A_flat[i * params.n + j];
      }
    }
  }

  if (node["B"]) {
    params.B.resize(params.n, params.m);
    std::vector<real_t> B_flat;
    node["B"].as<std::vector<real_t>>(B_flat);
    PARAM_ASSERT(
        B_flat.size() == params.n * params.m,
        "B matrix size is not correct, expected size: %d, actual size: %d",
        params.n * params.m, B_flat.size());
    for (size_t i = 0; i < params.n; ++i) {
      for (size_t j = 0; j < params.m; ++j) {
        params.B(i, j) = B_flat[i * params.m + j];
      }
    }
  }

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

  params.x_min_.resize(params.n);
  std::vector<real_t> x_min;
  node["x_min"].as<std::vector<real_t>>(x_min);
  PARAM_ASSERT(x_min.size() == params.n,
               "x_min size is not correct, expected size: %d, actual size: %d",
               params.n, x_min.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.x_min_(i) = x_min[i];
  }

  params.x_max_.resize(params.n);
  std::vector<real_t> x_max;
  node["x_max"].as<std::vector<real_t>>(x_max);
  PARAM_ASSERT(x_max.size() == params.n,
               "x_max size is not correct, expected size: %d, actual size: %d",
               params.n, x_max.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.x_max_(i) = x_max[i];
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

Mpc::Mpc(const Params& params)
{
  setParams(params);

  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  qp_.setOptions(op);
}

bool Mpc::solve(const StateSeq& x_ref_seq, const StateVec& x0, bool force_init)
{
  PARAM_ASSERT(x_ref_seq.size() == params_.horizon,
               "x_ref_seq size must be equal to horizon");
  PARAM_ASSERT(x0.size() == params_.n, "x0 size must be equal to n");

  x0_ = x0;
  calcG(x_ref_seq);
  calcLbAUbA();

  qpOASES::returnValue ret;

  qpOASES::int_t nWSR = params_.max_iter;
  if (!can_hot_start_ || force_init) {
    ret = qp_.init(H_.data(), g_.data(), A_.data(), lb_.data(), ub_.data(),
                   lbA_.data(), ubA_.data(), nWSR);
    can_hot_start_ = true;
  } else {
    ret = qp_.hotstart(g_.data(), lb_.data(), ub_.data(), lbA_.data(),
                       ub_.data(), nWSR);
  }

  if (ret != qpOASES::SUCCESSFUL_RETURN) {
    solved_ = false;
  } else {
    qp_.getPrimalSolution(U_bar_.data());
    solved_ = true;
  }

  return solved_;
}

void Mpc::getCtrl(InputVec& u, size_t forward_steps) const
{
  if (forward_steps >= params_.horizon) {
    forward_steps = params_.horizon - 1;
  }

  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u = U_bar_.segment(forward_steps * params_.m, params_.m);
}

void Mpc::getCtrlSeq(CtrlSeq& u_seq) const
{
  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u_seq.clear();
  for (size_t i = 0; i < params_.horizon; ++i) {
    u_seq.emplace_back(U_bar_.segment(i * params_.m, params_.m));
  }
}

void Mpc::getPredStateSeq(StateSeq& x_seq) const
{
  if (!solved_) {
    PARAM_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  Eigen::VectorX<real_t> x_bar = S_bar_ * U_bar_ + T_bar_ * x0_;
  x_seq.clear();

  for (size_t i = 0; i < params_.horizon; ++i) {
    x_seq.emplace_back(x_bar.segment(i * params_.n, params_.n));
  }
}

void Mpc::setParams(const Params& params)
{
  PARAM_ASSERT(params.n > 0, "The number of states must be greater than 0");
  PARAM_ASSERT(params.m > 0, "The number of inputs must be greater than 0");
  PARAM_ASSERT(params.horizon > 0,
               "The prediction horizon must be greater than 0");
  PARAM_ASSERT(params.max_iter > 0,
               "The maximum number of iterations must be greater than 0");
  PARAM_ASSERT(params.A.rows() == params.n && params.A.cols() == params.n,
               "A must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.B.rows() == params.n && params.B.cols() == params.m,
               "B must be a matrix of size (n, m)");
  PARAM_ASSERT(params.Q.rows() == params.n && params.Q.cols() == params.n,
               "Q must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.R.rows() == params.m && params.R.cols() == params.m,
               "R must be a square matrix of size (m, m)");
  PARAM_ASSERT(params.P.rows() == params.n && params.P.cols() == params.n,
               "P must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.x_min_.size() == params.n,
               "x_min_ size must be equal to n");
  PARAM_ASSERT(params.x_max_.size() == params.n,
               "x_max_ size must be equal to n");
  PARAM_ASSERT(params.u_min_.size() == params.m,
               "u_min_ size must be equal to m");
  PARAM_ASSERT(params.u_max_.size() == params.m,
               "u_max_ size must be equal to m");

  params_ = params;

  U_bar_ = Eigen::VectorX<real_t>::Zero(params_.m * params_.horizon);

  calcSBar();

  calcTBar();

  calcQBar();

  calcRBar();

  calcH();

  calcA();

  calcLbUb();

  can_hot_start_ = false;
}

void Mpc::calcSBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  const size_t& m = params_.m;
  S_bar_ = Eigen::MatrixX<real_t>::Zero(n * N, m * N);
  Eigen::MatrixX<real_t> A_i_x_B = params_.B;
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = 0; j < N - i; ++j) {
      S_bar_.block((i + j) * n, j * m, n, m) = A_i_x_B;
    }

    if (i < N - 1) {
      A_i_x_B = params_.A * A_i_x_B;
    }
  }
}

void Mpc::calcTBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  T_bar_ = Eigen::MatrixX<real_t>::Zero(n * N, n);
  Eigen::MatrixX<real_t> A_i = Eigen::MatrixX<real_t>::Identity(n, n);
  for (size_t i = 0; i < N; ++i) {
    A_i = params_.A * A_i;
    T_bar_.block(i * n, 0, n, n) = A_i;
  }
}

void Mpc::calcQBar(void)
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

void Mpc::calcRBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& m = params_.m;
  R_bar_.resize(m * N);
  const auto& R = params_.R;
  R_bar_.diagonal() = R.diagonal().replicate(N, 1);
}

void Mpc::calcH(void)
{
  H_ = S_bar_.transpose() * Q_bar_ * S_bar_;
  H_.diagonal() += R_bar_.diagonal();
}

void Mpc::calcG(const StateSeq& x_ref_seq)
{
  Eigen::VectorX<real_t> x_ref =
      Eigen::VectorX<real_t>::Zero(params_.n * params_.horizon);
  for (size_t i = 0; i < params_.horizon; ++i) {
    PARAM_ASSERT(x_ref_seq[i].size() == params_.n,
                 "x_ref_seq[%d] size must be equal to n", i);
    x_ref.segment(i * params_.n, params_.n) = x_ref_seq[i];
  }
  g_ = S_bar_.transpose() * Q_bar_ * (T_bar_ * x0_ - x_ref);
}

void Mpc::calcLbUb(void)
{
  lb_ = params_.u_min_.replicate(params_.horizon, 1);
  ub_ = params_.u_max_.replicate(params_.horizon, 1);
}

void Mpc::calcLbAUbA(void)
{
  size_t& N = params_.horizon;
  size_t& n = params_.n;
  size_t& m = params_.m;

  Eigen::VectorX<real_t> x_bar_min = params_.x_min_.replicate(N, 1);
  Eigen::VectorX<real_t> x_bar_max = params_.x_max_.replicate(N, 1);

  lbA_ = x_bar_min - T_bar_ * x0_;
  ubA_ = x_bar_max - T_bar_ * x0_;
}
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils

#endif /* HAS_QPOASES */