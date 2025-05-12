/**
 *******************************************************************************
 * @file mpc.cpp
 * @brief Model Predictive Control (MPC) controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-10
 * @author Caikunzhen
 * @details
 * 1. Complete the mpc.cpp
 *******************************************************************************
 * @attention
 * This file will only compile when the qpOASES library is found.
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
  params.max_cost_time = node["max_cost_time"].as<qpOASES::real_t>();

  if (node["A"]) {
    params.A.resize(params.n, params.n);
    std::vector<real_t> A_flat = node["A"].as<std::vector<real_t>>();
    RU_ASSERT(
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
    std::vector<real_t> B_flat = node["B"].as<std::vector<real_t>>();
    RU_ASSERT(
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
  std::vector<real_t> Q_diag = node["Q_diag"].as<std::vector<real_t>>();
  RU_ASSERT(Q_diag.size() == params.n,
            "Q matrix size is not correct, expected size: %d, actual size: %d",
            params.n, Q_diag.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.Q.diagonal()[i] = Q_diag[i];
  }

  params.P.resize(params.n);
  std::vector<real_t> P_diag = node["P_diag"].as<std::vector<real_t>>();
  RU_ASSERT(P_diag.size() == params.n,
            "P matrix size is not correct, expected size: %d, actual size: %d",
            params.n, P_diag.size());
  for (size_t i = 0; i < params.n; ++i) {
    params.P.diagonal()[i] = P_diag[i];
  }

  params.R.resize(params.m);
  std::vector<real_t> R_diag = node["R_diag"].as<std::vector<real_t>>();
  RU_ASSERT(R_diag.size() == params.m,
            "R matrix size is not correct, expected size: %d, actual size: %d",
            params.m, R_diag.size());
  for (size_t i = 0; i < params.m; ++i) {
    params.R.diagonal()[i] = R_diag[i];
  }

  params.state_bound = node["state_bound"].as<bool>();
  if (params.state_bound) {
    params.x_min.resize(params.n);
    std::vector<real_t> x_min = node["x_min"].as<std::vector<real_t>>();
    RU_ASSERT(x_min.size() == params.n,
              "x_min size is not correct, expected size: %d, actual size: %d",
              params.n, x_min.size());
    for (size_t i = 0; i < params.n; ++i) {
      params.x_min(i) = x_min[i];
    }

    params.x_max.resize(params.n);
    std::vector<real_t> x_max = node["x_max"].as<std::vector<real_t>>();
    RU_ASSERT(x_max.size() == params.n,
              "x_max size is not correct, expected size: %d, actual size: %d",
              params.n, x_max.size());
    for (size_t i = 0; i < params.n; ++i) {
      params.x_max(i) = x_max[i];
    }
  }

  params.input_bound = node["input_bound"].as<bool>();
  if (params.input_bound) {
    params.u_min.resize(params.m);
    std::vector<real_t> u_min = node["u_min"].as<std::vector<real_t>>();
    RU_ASSERT(u_min.size() == params.m,
              "u_min size is not correct, expected size: %d, actual size: %d",
              params.m, u_min.size());
    for (size_t i = 0; i < params.m; ++i) {
      params.u_min(i) = u_min[i];
    }

    params.u_max.resize(params.m);
    std::vector<real_t> u_max = node["u_max"].as<std::vector<real_t>>();
    RU_ASSERT(u_max.size() == params.m,
              "u_max size is not correct, expected size: %d, actual size: %d",
              params.m, u_max.size());
    for (size_t i = 0; i < params.m; ++i) {
      params.u_max(i) = u_max[i];
    }
  }
}

Mpc::Mpc(const Params& params)
{
  RU_ASSERT(params.n > 0, "The number of states must be greater than 0");
  RU_ASSERT(params.m > 0, "The number of inputs must be greater than 0");
  RU_ASSERT(params.horizon > 0,
            "The prediction horizon must be greater than 0");

  params_.n = params.n;
  params_.m = params.m;
  params_.horizon = params.horizon;
  params_.state_bound = params.state_bound;
  params_.input_bound = params.input_bound;
  const size_t& n = params_.n;
  const size_t& m = params_.m;
  const size_t& N = params_.horizon;
  U_ = Eigen::VectorX<real_t>::Zero(m * N);
  S_bar_ = Eigen::MatrixX<real_t>::Zero(n * N, m * N);
  T_bar_ = Eigen::MatrixX<real_t>::Zero(n * N, n);
  Q_bar_.resize(n * N);
  R_bar_.resize(m * N);
  setParams(params);

  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  if (params_.state_bound) {
    qp_ = qpOASES::QProblem(m * N, n * N, qpOASES::HST_SEMIDEF);
  } else {
    qp_ = qpOASES::QProblem(m * N, 0, qpOASES::HST_SEMIDEF);
  }
  qp_.setOptions(op);
}

bool Mpc::solve(const StateSeq& x_ref_seq, const StateVec& x0, bool force_init)
{
  RU_ASSERT(x_ref_seq.size() == params_.horizon,
            "x_ref_seq size must be equal to horizon");
  RU_ASSERT(x0.size() == params_.n, "x0 size must be equal to n");

  x0_ = x0;
  calcG(x_ref_seq);
  if (params_.state_bound) {
    calcLbAUbA();
    lbA_ptr_ = lbA_.data();
    ubA_ptr_ = ubA_.data();
  }

  qpOASES::int_t nWSR = params_.max_iter;
  real_t cputime = params_.max_cost_time;
  if (params_.max_cost_time <= 0) {
    cputime = 100;
  }

  /**
   * solve the QP problem
   *
   * The QP problem is defined as:
   *
   * \f[
   * \underset{^{qp}U}{\min}J = \frac{1}{2}\ ^{qp}U^T\ ^{qp}H\ ^{qp}U
   * +\ ^{qp}U^T\ ^{qp}g
   * \f]
   *
   * s.t.
   *
   * \f[
   * ^{qp}b_{min} \leq\ ^{qp}A\ ^{qp}U \leq\ ^{qp}b_{max}
   * \f]
   *
   * \f[
   * ^{qp}U_{min} \leq\ ^{qp}U \leq\ ^{qp}U_{max}
   * \f]
   */
  if (!data_.can_hot_start || force_init) {
    data_.qp_ret = qp_.init(H_.data(), g_.data(), A_ptr_, lb_ptr_, ub_ptr_,
                            lbA_ptr_, ubA_ptr_, nWSR, &cputime);
    data_.can_hot_start = true;
  } else {
    data_.qp_ret = qp_.hotstart(g_.data(), lb_ptr_, ub_ptr_, lbA_ptr_, ubA_ptr_,
                                nWSR, &cputime);
  }

  data_.iter = nWSR;
  data_.cost_time = cputime;
  if (data_.qp_ret != qpOASES::SUCCESSFUL_RETURN) {
    data_.solved = false;
  } else {
    qp_.getPrimalSolution(U_.data());
    data_.solved = true;
  }

  return data_.solved;
}

void Mpc::getCtrl(InputVec& u, size_t forward_steps) const
{
  if (forward_steps >= params_.horizon) {
    forward_steps = params_.horizon - 1;
  }

  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u = U_.segment(forward_steps * params_.m, params_.m);
}

void Mpc::getCtrlSeq(CtrlSeq& u_seq) const
{
  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u_seq.clear();
  for (size_t i = 0; i < params_.horizon; ++i) {
    u_seq.emplace_back(U_.segment(i * params_.m, params_.m));
  }
}

void Mpc::getPredStateSeq(StateSeq& x_seq) const
{
  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  Eigen::VectorX<real_t> x_bar = S_bar_ * U_ + T_bar_ * x0_;
  x_seq.clear();

  for (size_t i = 0; i < params_.horizon; ++i) {
    x_seq.emplace_back(x_bar.segment(i * params_.n, params_.n));
  }
}

void Mpc::setParams(const Params& params)
{
  RU_ASSERT(params.max_iter > 0,
            "The maximum number of iterations must be greater than 0");
  RU_ASSERT(params.A.rows() == params_.n && params.A.cols() == params_.n,
            "A must be a square matrix of size (n, n)");
  RU_ASSERT(params.B.rows() == params_.n && params.B.cols() == params_.m,
            "B must be a matrix of size (n, m)");
  RU_ASSERT(params.Q.rows() == params_.n && params.Q.cols() == params_.n,
            "Q must be a square matrix of size (n, n)");
  RU_ASSERT(params.R.rows() == params_.m && params.R.cols() == params_.m,
            "R must be a square matrix of size (m, m)");
  RU_ASSERT(params.P.rows() == params_.n && params.P.cols() == params_.n,
            "P must be a square matrix of size (n, n)");
  if (params_.state_bound) {
    RU_ASSERT(params.x_min.size() == params_.n,
              "x_min size must be equal to n");
    RU_ASSERT(params.x_max.size() == params_.n,
              "x_max size must be equal to n");
  }
  if (params_.input_bound) {
    RU_ASSERT(params.u_min.size() == params_.m,
              "u_min size must be equal to m");
    RU_ASSERT(params.u_max.size() == params_.m,
              "u_max size must be equal to m");
  }

  size_t n = params_.n;
  size_t m = params_.m;
  size_t horizon = params_.horizon;
  bool state_bound = params_.state_bound;
  bool input_bound = params_.input_bound;
  params_ = params;
  params_.n = n;
  params_.m = m;
  params_.horizon = horizon;
  params_.state_bound = state_bound;
  params_.input_bound = input_bound;

  calcSBar();

  calcTBar();

  calcQBar();

  calcRBar();

  calcH();

  if (params_.state_bound) {
    calcA();
    A_ptr_ = A_.data();
  }

  if (params_.input_bound) {
    calcLbUb();
    lb_ptr_ = lb_.data();
    ub_ptr_ = ub_.data();
  }

  data_.can_hot_start = false;
}

void Mpc::calcSBar(void)
{
  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  const size_t& m = params_.m;
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
    RU_ASSERT(x_ref_seq[i].size() == params_.n,
              "x_ref_seq[%d] size must be equal to n", i);
    x_ref.segment(i * params_.n, params_.n) = x_ref_seq[i];
  }
  g_ = S_bar_.transpose() * Q_bar_ * (T_bar_ * x0_ - x_ref);
}

void Mpc::calcLbUb(void)
{
  lb_ = params_.u_min.replicate(params_.horizon, 1);
  ub_ = params_.u_max.replicate(params_.horizon, 1);
}

void Mpc::calcLbAUbA(void)
{
  size_t& N = params_.horizon;
  size_t& n = params_.n;
  size_t& m = params_.m;

  Eigen::VectorX<real_t> x_bar_min = params_.x_min.replicate(N, 1);
  Eigen::VectorX<real_t> x_bar_max = params_.x_max.replicate(N, 1);

  lbA_ = x_bar_min - T_bar_ * x0_;
  ubA_ = x_bar_max - T_bar_ * x0_;
}
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
#endif  // HAS_QPOASES