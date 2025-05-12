/**
 *******************************************************************************
 * @file      : ommpc.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
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
  params.max_cost_time = node["max_cost_time"].as<qpOASES::real_t>();
  params.dt = node["dt"].as<real_t>();

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

Ommpc::Ommpc(const Params& params)
    : qp_(params.m * params.horizon, qpOASES::HST_SEMIDEF)
{
  RU_ASSERT(params.n > 0, "The number of states must be greater than 0");
  RU_ASSERT(params.m > 0, "The number of inputs must be greater than 0");
  RU_ASSERT(params.horizon > 0,
            "The prediction horizon must be greater than 0");
  RU_ASSERT(params.dt > 0, "The time step must be greater than 0");

  params_.n = params.n;
  params_.m = params.m;
  params_.horizon = params.horizon;
  params_.input_bound = params.input_bound;
  params_.dt = params.dt;
  const size_t& n = params_.n;
  const size_t& m = params_.m;
  const size_t& N = params_.horizon;
  S_bar_ = Eigen::MatrixX<real_t>::Zero(N * n, N * m);
  T_bar_ = Eigen::MatrixX<real_t>::Zero(N * n, n);
  dU_ = Eigen::MatrixX<real_t>::Zero(N * m, 1);
  Q_bar_.resize(N * n);
  R_bar_.resize(N * m);
  if (params_.input_bound) {
    lb_.resize(N * m);
    ub_.resize(N * m);
    lb_ptr_ = lb_.data();
    ub_ptr_ = ub_.data();
  }
  setParams(params);

  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_NONE;
  qp_.setOptions(op);
}

bool Ommpc::solve(const StateSeq& x_ref_seq, const CtrlSeq& u_ref_seq,
                  const StateVec& x0)
{
  RU_ASSERT(x_ref_seq.size() == params_.horizon + 1,
            "x_ref_seq size must be equal to horizon + 1");
  RU_ASSERT(u_ref_seq.size() == params_.horizon,
            "u_ref_seq size must be equal to horizon");
  RU_ASSERT(x0.dim() == params_.n, "x0 size must be equal to n");

  const size_t& N = params_.horizon;
  const size_t& n = params_.n;
  const size_t& m = params_.m;

  if (x_ref_seq_.empty()) {
    x_ref_seq_ = x_ref_seq;
  } else {
    for (size_t i = 0; i < N + 1; ++i) {
      x_ref_seq_[i].copyData(x_ref_seq[i]);
    }
  }
  u_ref_seq_ = u_ref_seq;

  Eigen::MatrixX<real_t> F_x_i, F_u_i;
  Eigen::MatrixX<real_t> F_x_mul = Eigen::MatrixX<real_t>::Identity(n, n);
  for (size_t i = 0; i < N; ++i) {
    RU_ASSERT(x_ref_seq[i].dim() == n,
              "x_ref_seq[%zu] dim must be equal to n, expected %zu, got %zu", i,
              n, x_ref_seq[i].dim());
    RU_ASSERT(u_ref_seq[i].size() == m,
              "u_ref_seq[%zu] size must be equal to m, expected %zu, got %zu",
              i, m, u_ref_seq[i].size());

    getJacobian(x_ref_seq[i], u_ref_seq[i], F_x_i, F_u_i);
    for (size_t j = 0; j < i; ++j) {
      S_bar_.block(i * n, j * m, n, m) =
          F_x_i * S_bar_.block((i - 1) * n, j * m, n, m);
    }
    S_bar_.block(i * n, i * m, n, m) = F_u_i;
    F_x_mul = F_x_i * F_x_mul;
    T_bar_.block(i * n, 0, n, n) = F_x_mul;
    if (params_.input_bound) {
      lb_.segment(i * m, m) = params_.u_min - u_ref_seq[i];
      ub_.segment(i * m, m) = params_.u_max - u_ref_seq[i];
    }
  }

  Eigen::MatrixX<real_t> tmp = S_bar_.transpose() * Q_bar_;
  H_ = tmp * S_bar_;
  H_.diagonal() += R_bar_.diagonal();
  dx0_ = x0 - x_ref_seq[0];
  g_ = tmp * T_bar_ * dx0_;

  /**
   * solve the QP problem
   *
   * The QP problem is defined as:
   *
   * \f[
   * \underset{\delta ^{qp}U}{\min}J =
   * \frac{1}{2}\delta\ ^{qp}U^T\ ^{qp}H\delta\ ^{qp}U +\delta\ ^{qp}U^T\ ^{qp}g
   * \f]
   *
   * s.t.
   *
   * \f[
   * ^{qp}b_{min} \leq\ ^{qp}A\delta\ ^{qp}U \leq\ ^{qp}b_{max}
   * \f]
   *
   * \f[
   * \delta\ ^{qp}U_{min} \leq\delta\ ^{qp}U \leq\delta\ ^{qp}U_{max}
   * \f]
   */
  qpOASES::int_t nWSR = params_.max_iter;
  real_t cputime = params_.max_cost_time;
  if (params_.max_cost_time <= 0) {
    cputime = 100;
  }
  data_.qp_ret =
      qp_.init(H_.data(), g_.data(), lb_ptr_, ub_ptr_, nWSR, &cputime);

  data_.iter = nWSR;
  data_.cost_time = cputime;
  if (data_.qp_ret != qpOASES::SUCCESSFUL_RETURN) {
    data_.solved = false;
  } else {
    qp_.getPrimalSolution(dU_.data());
    data_.solved = true;
  }

  return data_.solved;
}

void Ommpc::getCtrl(InputVec& u, size_t forward_steps) const
{
  if (forward_steps >= params_.horizon) {
    forward_steps = params_.horizon - 1;
  }

  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u = dU_.segment(forward_steps * params_.m, params_.m) +
      u_ref_seq_[forward_steps];
}

void Ommpc::getCtrlSeq(CtrlSeq& u_seq) const
{
  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  u_seq.clear();
  for (size_t i = 0; i < params_.horizon; ++i) {
    u_seq.emplace_back(dU_.segment(i * params_.m, params_.m) + u_ref_seq_[i]);
  }
}

void Ommpc::getPredStateSeq(StateSeq& x_seq) const
{
  if (!data_.solved) {
    RU_ASSERT(false, "MPC has not been solved yet");
    return;
  }
  Eigen::VectorX<real_t> dx_bar = S_bar_ * dU_ + T_bar_ * dx0_;
  x_seq.clear();

  for (size_t i = 0; i < params_.horizon; ++i) {
    x_seq.emplace_back(x_ref_seq_[i + 1] +
                       dx_bar.segment(i * params_.n, params_.n));
  }
}

void Ommpc::setParams(const Params& params)
{
  RU_ASSERT(params.max_iter > 0,
            "The maximum number of iterations must be greater than 0");
  RU_ASSERT(params.f, "f must be a non-null function");
  RU_ASSERT(params.df_dx, "df_dx must be a non-null function");
  RU_ASSERT(params.df_du, "df_du must be a non-null function");
  RU_ASSERT(params.Q.rows() == params_.n && params.Q.cols() == params_.n,
            "Q must be a square matrix of size (n, n)");
  RU_ASSERT(params.R.rows() == params_.m && params.R.cols() == params_.m,
            "R must be a square matrix of size (m, m)");
  RU_ASSERT(params.P.rows() == params_.n && params.P.cols() == params_.n,
            "P must be a square matrix of size (n, n)");
  if (params_.input_bound) {
    RU_ASSERT(params.u_min.size() == params_.m,
              "u_min size must be equal to m");
    RU_ASSERT(params.u_max.size() == params_.m,
              "u_max size must be equal to m");
  }

  size_t n = params_.n;
  size_t m = params_.m;
  size_t horizon = params_.horizon;
  real_t dt = params_.dt;
  bool input_bound = params_.input_bound;
  params_ = params;
  params_.dt = dt;
  params_.n = n;
  params_.m = m;
  params_.horizon = horizon;
  params_.dt = dt;
  params_.input_bound = input_bound;

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
  RU_ASSERT(prim_m.dim() == delta.size(),
            "Manifold dimension mismatch, expected %zu, got %zu", prim_m.dim(),
            delta.size());

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
      RU_ASSERT(false, "Unsupported manifold type");
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
  F_x = G_x + params_.dt * G_f * params_.df_dx(x_ref, u_ref);
  F_u = params_.dt * G_f * params_.df_du(x_ref, u_ref);
}
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
#endif  // HAS_QPOASES