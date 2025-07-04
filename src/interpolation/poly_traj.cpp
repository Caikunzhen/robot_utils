/**
 *******************************************************************************
 * @file poly_traj.cpp
 * @brief
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-05-27
 * @author Caikunzhen
 * @details
 * 1. Complete the poly_traj.cpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/interpolation/poly_traj.hpp"

#include <cmath>

#include "robot_utils/core/math_tools.hpp"
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

template <typename T>
static T PolyCoeffDerivate(size_t n, size_t n_d);

#ifdef HAS_NLOPT
bool SmoothedL1(double mu, double x, double& f, double& df);
#endif  // HAS_NLOPT
/* Exported function definitions ---------------------------------------------*/

template <typename T>
VectorX<T> Poly<T>::operator()(const T& t, size_t n_diff) const
{
  VectorX<T> x(coeff_.rows());
  x.setZero();
  if (n_diff > coeff_.cols()) {
    return x;
  }
  for (size_t i = coeff_.cols() - 1; i >= n_diff; --i) {
    x = x * t + coeff_.col(i) * PolyCoeffDerivate<T>(i, n_diff);
    if (i == 0) {
      break;
    }
  }
  return x;
}

template <typename T>
PolyTraj<T>::PolyTraj(size_t dim, int cfg_mask) : dim_(dim), cfg_mask_(cfg_mask)
{
  polys_.emplace_back(MatrixX<T>::Zero(dim, 1));
  dts_.resize(1);
  dts_[0] = 0;
}

template <typename T>
VectorX<T> PolyTraj<T>::operator()(T t, size_t n_diff) const
{
  size_t idx = 0;
  T dt = 0;
  getPolyIdxAndDt(t, idx, dt);
  T dt_n = 1;
  if (is_time_scale_) {
    for (size_t i = 0; i < n_diff; ++i) {
      dt_n *= dts_[idx];
    }
    dt /= dts_[idx];
  }
  return polys_[idx](dt, n_diff) / dt_n;
}

template <typename T>
void PolyTraj<T>::getPolyIdxAndDt(const T& t, size_t& idx, T& dt) const
{
  T t_p = t - t0_;
  if (cfg_mask_ & kCfgMaskStartClamp) {
    t_p = std::max<T>(t_p, 0);
  }
  for (idx = 0; idx < dts_.size(); ++idx) {
    if (t_p >= dts_[idx]) {
      t_p -= dts_[idx];
    } else {
      dt = t_p;
      break;
    }
  }

  if (idx == dts_.size()) {
    idx = dts_.size() - 1;
    if (cfg_mask_ & kCfgMaskEndClamp) {
      dt = dts_[idx];
    } else {
      dt = t_p;
    }
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
void PolyTrajOpt<T, Dim, EnergyOrder>::optimize(const State& x0,
                                                const State& xf,
                                                const Waypoints& xi, T t0,
                                                const std::vector<T>& dts,
                                                PolyTraj<T>& traj, T* J,
                                                VectorX<T>* partial_grad_by_dts)
{
  RU_ASSERT(xi.size() == dts.size() - 1,
            "xi must have the same number of columns as size of durs - 1");

  if (M_ != dts.size()) {
    M_ = dts.size();
    C_T_ = getCT();
  }
  dts_ = Eigen::Map<const VectorX<T>>(dts.data(), dts.size());
  dt2s_ = dts_.cwiseProduct(dts_);
  dt3s_ = dt2s_.cwiseProduct(dts_);
  dt4s_ = dt3s_.cwiseProduct(dts_);
  if constexpr (EnergyOrder > 2) {
    dt5s_ = dt4s_.cwiseProduct(dts_);
    dt6s_ = dt5s_.cwiseProduct(dts_);
  }
  if constexpr (EnergyOrder > 3) {
    dt7s_ = dt6s_.cwiseProduct(dts_);
    dt8s_ = dt7s_.cwiseProduct(dts_);
  }
  MatrixX<T> H = getH();
  MatrixX<T> R = C_T_.transpose() * H * C_T_;
  const size_t df_dim = 2 * _R + M_ - 1;
  const size_t dp_dim = (M_ - 1) * (_R - 1);
  MatrixX<T> Df(df_dim, Dim);
  for (size_t i = 0; i < Dim; ++i) {
    Df.col(i).segment(0, _R) = x0.row(i);
    Df.col(i).segment(_R, _R) = xf.row(i);
    for (size_t j = 0; j < M_ - 1; ++j) {
      Df(2 * _R + j, i) = xi[j][i];
    }
  }
  MatrixX<T> DfDp(df_dim + dp_dim, Dim);
  if (dp_dim != 0) {
    MatrixX<T> R_PF = R.block(df_dim, 0, dp_dim, df_dim);
    MatrixX<T> R_PP = R.block(df_dim, df_dim, dp_dim, dp_dim);
    MatrixX<T> Dp = R_PP.llt().solve(-R_PF * Df);
    DfDp << Df, Dp;
  } else {
    DfDp << Df;
  }
  MatrixX<T> D = C_T_ * DfDp;
  std::vector<MatrixX<T>> coeffs;
  coeffs.reserve(M_);
  Eigen::Matrix<T, 2 * _R, Dim> d;
  for (size_t i = 0; i < M_; ++i) {
    d = D.block(2 * i * _R, 0, 2 * _R, Dim);
    auto coeff = getAiInv(i) * d;
    coeffs.emplace_back(coeff.transpose());
  }
  traj.setTraj(t0, coeffs, dts);

  if (partial_grad_by_dts) {
    MatrixX<T> R_PP_inv_R_PF;
    if (dp_dim != 0) {
      MatrixX<T> R_PP = R.block(df_dim, df_dim, dp_dim, dp_dim);
      MatrixX<T> R_PF = R.block(df_dim, 0, dp_dim, df_dim);
      R_PP_inv_R_PF = R_PP.llt().solve(R_PF);
    }
    partial_grad_by_dts->resize(M_);
    *partial_grad_by_dts = getEnergyPartialGradByDt(R_PP_inv_R_PF, Df);
  }

  if (J) {
    *J = 0;
    for (size_t i = 0; i < Dim; ++i) {
      *J += DfDp.col(i).dot(R * DfDp.col(i));
    }
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
void PolyTrajOpt<T, Dim, EnergyOrder>::simpleTimeAllocate(
    const State& x0, const State& xf, const Waypoints& xi, T max_vel, T max_acc,
    std::vector<T>& dts) const
{
  RU_ASSERT(max_vel > 0 && max_acc > 0,
            "max_vel and max_acc must be greater than 0");

  const size_t M = xi.size() + 1;
  dts.resize(M);
  VectorX<T> x_i = x0.col(0);
  for (size_t i = 0; i < M; ++i) {
    double dist = 0;
    if (i == M - 1) {
      dist = (xf.col(0) - x_i).norm();
    } else {
      dist = (xi[i] - x_i).norm();
    }

    // Acceleration and deceleration distance
    T d_acc = max_vel * max_vel / max_acc;
    if (dist < d_acc) {
      dts[i] = std::sqrt(dist / max_acc);
    } else {
      T d_vel = dist - d_acc;
      T t_acc = 2 * max_vel / max_acc;
      T t_vel = d_vel / max_vel;
      dts[i] = t_acc + t_vel;
    }
    if (i < M - 1) {
      x_i = xi[i];
    }
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder>
PolyTrajOpt<T, Dim, EnergyOrder>::getQDur2R_1(void) const
{
  if constexpr (EnergyOrder == 2) {
    Eigen::Matrix<T, 4, 4> Qi;
    Qi.setZero();
    Qi(2, 2) = 4;
    Qi(2, 3) = Qi(3, 2) = 6;
    Qi(3, 3) = 12;
    return Qi;
  } else if constexpr (EnergyOrder == 3) {
    Eigen::Matrix<T, 6, 6> Qi;
    Qi.setZero();
    Qi(3, 3) = 36;
    Qi(3, 4) = Qi(4, 3) = 72;
    Qi(3, 5) = Qi(5, 3) = 120;
    Qi(4, 4) = 192;
    Qi(4, 5) = Qi(5, 4) = 360;
    Qi(5, 5) = 720;
    return Qi;
  } else {
    Eigen::Matrix<T, 8, 8> Qi;
    Qi.setZero();
    Qi(4, 4) = 576;
    Qi(4, 5) = Qi(5, 4) = 1440;
    Qi(4, 6) = Qi(6, 4) = 2880;
    Qi(5, 5) = 4800;
    Qi(4, 7) = Qi(7, 4) = 5040;
    Qi(5, 6) = Qi(6, 5) = 10800;
    Qi(5, 7) = Qi(7, 5) = 20160;
    Qi(6, 6) = 25920;
    Qi(6, 7) = Qi(7, 6) = 50400;
    Qi(7, 7) = 100800;
    return Qi;
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder>
PolyTrajOpt<T, Dim, EnergyOrder>::getAiInv(size_t i) const
{
  if constexpr (EnergyOrder == 2) {
    Eigen::Matrix<T, 4, 4> Ai_inv;
    Ai_inv.setZero();
    Ai_inv(0, 0) = 1;
    Ai_inv(1, 1) = dts_[i];
    Ai_inv(2, 0) = -3;
    Ai_inv(2, 1) = -2 * dts_[i];
    Ai_inv(2, 2) = 3;
    Ai_inv(2, 3) = -dts_[i];
    Ai_inv(3, 0) = 2;
    Ai_inv(3, 1) = dts_[i];
    Ai_inv(3, 2) = -2;
    Ai_inv(3, 3) = dts_[i];
    return Ai_inv;
  } else if constexpr (EnergyOrder == 3) {
    Eigen::Matrix<T, 6, 6> Ai_inv;
    Ai_inv.setZero();
    Ai_inv(0, 0) = 1;
    Ai_inv(1, 1) = dts_[i];
    Ai_inv(2, 2) = dt2s_[i] / 2;
    Ai_inv(3, 0) = -10;
    Ai_inv(3, 1) = -6 * dts_[i];
    Ai_inv(3, 2) = -3 * dt2s_[i] / 2;
    Ai_inv(3, 3) = 10;
    Ai_inv(3, 4) = -4 * dts_[i];
    Ai_inv(3, 5) = dt2s_[i] / 2;
    Ai_inv(4, 0) = 15;
    Ai_inv(4, 1) = 8 * dts_[i];
    Ai_inv(4, 2) = 3 * dt2s_[i] / 2;
    Ai_inv(4, 3) = -15;
    Ai_inv(4, 4) = 7 * dts_[i];
    Ai_inv(4, 5) = -dt2s_[i];
    Ai_inv(5, 0) = -6;
    Ai_inv(5, 1) = -3 * dts_[i];
    Ai_inv(5, 2) = -dt2s_[i] / 2;
    Ai_inv(5, 3) = 6;
    Ai_inv(5, 4) = -3 * dts_[i];
    Ai_inv(5, 5) = dt2s_[i] / 2;
    return Ai_inv;
  } else {
    Eigen::Matrix<T, 8, 8> Ai_inv;
    Ai_inv.setZero();
    Ai_inv(0, 0) = 1;
    Ai_inv(1, 1) = dts_[i];
    Ai_inv(2, 2) = dt2s_[i] / 2;
    Ai_inv(3, 3) = dt3s_[i] / 6;
    Ai_inv(4, 0) = -35;
    Ai_inv(4, 1) = -20 * dts_[i];
    Ai_inv(4, 2) = -5 * dt2s_[i];
    Ai_inv(4, 3) = -2 * dt3s_[i] / 3;
    Ai_inv(4, 4) = 35;
    Ai_inv(4, 5) = -15 * dts_[i];
    Ai_inv(4, 6) = 5 * dt2s_[i] / 2;
    Ai_inv(4, 7) = -dt3s_[i] / 6;
    Ai_inv(5, 0) = 84;
    Ai_inv(5, 1) = 45 * dts_[i];
    Ai_inv(5, 2) = 10 * dt2s_[i];
    Ai_inv(5, 3) = dt3s_[i];
    Ai_inv(5, 4) = -84;
    Ai_inv(5, 5) = 39 * dts_[i];
    Ai_inv(5, 6) = -7 * dt2s_[i];
    Ai_inv(5, 7) = dt3s_[i] / 2;
    Ai_inv(6, 0) = -70;
    Ai_inv(6, 1) = -36 * dts_[i];
    Ai_inv(6, 2) = -15 * dt2s_[i] / 2;
    Ai_inv(6, 3) = -2 * dt3s_[i] / 3;
    Ai_inv(6, 4) = 70;
    Ai_inv(6, 5) = -34 * dts_[i];
    Ai_inv(6, 6) = 13 * dt2s_[i] / 2;
    Ai_inv(6, 7) = -dt3s_[i] / 2;
    Ai_inv(7, 0) = 20;
    Ai_inv(7, 1) = 10 * dts_[i];
    Ai_inv(7, 2) = 2 * dt2s_[i];
    Ai_inv(7, 3) = dt3s_[i] / 6;
    Ai_inv(7, 4) = -20;
    Ai_inv(7, 5) = 10 * dts_[i];
    Ai_inv(7, 6) = -2 * dt2s_[i];
    Ai_inv(7, 7) = dt3s_[i] / 6;
    return Ai_inv;
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder>
PolyTrajOpt<T, Dim, EnergyOrder>::getHi(size_t i) const
{
  if constexpr (EnergyOrder == 2) {
    Eigen::Matrix<T, 4, 4> Hi;
    Hi(0, 0) = 12 / dt3s_[i];
    Hi(0, 1) = 6 / dt2s_[i];
    Hi(0, 2) = -12 / dt3s_[i];
    Hi(0, 3) = 6 / dt2s_[i];
    Hi(1, 0) = 6 / dt2s_[i];
    Hi(1, 1) = 4 / dts_[i];
    Hi(1, 2) = -6 / dt2s_[i];
    Hi(1, 3) = 2 / dts_[i];
    Hi(2, 0) = -12 / dt3s_[i];
    Hi(2, 1) = -6 / dt2s_[i];
    Hi(2, 2) = 12 / dt3s_[i];
    Hi(2, 3) = -6 / dt2s_[i];
    Hi(3, 0) = 6 / dt2s_[i];
    Hi(3, 1) = 2 / dts_[i];
    Hi(3, 2) = -6 / dt2s_[i];
    Hi(3, 3) = 4 / dts_[i];
    return Hi;
  } else if constexpr (EnergyOrder == 3) {
    Eigen::Matrix<T, 6, 6> Hi;
    Hi(0, 0) = 720 / dt5s_[i];
    Hi(0, 1) = 360 / dt4s_[i];
    Hi(0, 2) = 60 / dt3s_[i];
    Hi(0, 3) = -720 / dt5s_[i];
    Hi(0, 4) = 360 / dt4s_[i];
    Hi(0, 5) = -60 / dt3s_[i];
    Hi(1, 0) = 360 / dt4s_[i];
    Hi(1, 1) = 192 / dt3s_[i];
    Hi(1, 2) = 36 / dt2s_[i];
    Hi(1, 3) = -360 / dt4s_[i];
    Hi(1, 4) = 168 / dt3s_[i];
    Hi(1, 5) = -24 / dt2s_[i];
    Hi(2, 0) = 60 / dt3s_[i];
    Hi(2, 1) = 36 / dt2s_[i];
    Hi(2, 2) = 9 / dts_[i];
    Hi(2, 3) = -60 / dt3s_[i];
    Hi(2, 4) = 24 / dt2s_[i];
    Hi(2, 5) = -3 / dts_[i];
    Hi(3, 0) = -720 / dt5s_[i];
    Hi(3, 1) = -360 / dt4s_[i];
    Hi(3, 2) = -60 / dt3s_[i];
    Hi(3, 3) = 720 / dt5s_[i];
    Hi(3, 4) = -360 / dt4s_[i];
    Hi(3, 5) = 60 / dt3s_[i];
    Hi(4, 0) = 360 / dt4s_[i];
    Hi(4, 1) = 168 / dt3s_[i];
    Hi(4, 2) = 24 / dt2s_[i];
    Hi(4, 3) = -360 / dt4s_[i];
    Hi(4, 4) = 192 / dt3s_[i];
    Hi(4, 5) = -36 / dt2s_[i];
    Hi(5, 0) = -60 / dt3s_[i];
    Hi(5, 1) = -24 / dt2s_[i];
    Hi(5, 2) = -3 / dts_[i];
    Hi(5, 3) = 60 / dt3s_[i];
    Hi(5, 4) = -36 / dt2s_[i];
    Hi(5, 5) = 9 / dts_[i];
    return Hi;
  } else {
    Eigen::Matrix<T, 8, 8> Hi;
    Hi(0, 0) = 100800 / dt7s_[i];
    Hi(0, 1) = 50400 / dt6s_[i];
    Hi(0, 2) = 10080 / dt5s_[i];
    Hi(0, 3) = 840 / dt4s_[i];
    Hi(0, 4) = -100800 / dt7s_[i];
    Hi(0, 5) = 50400 / dt6s_[i];
    Hi(0, 6) = -10080 / dt5s_[i];
    Hi(0, 7) = 840 / dt4s_[i];
    Hi(1, 0) = 50400 / dt6s_[i];
    Hi(1, 1) = 25920 / dt5s_[i];
    Hi(1, 2) = 5400 / dt4s_[i];
    Hi(1, 3) = 480 / dt3s_[i];
    Hi(1, 4) = -50400 / dt6s_[i];
    Hi(1, 5) = 24480 / dt5s_[i];
    Hi(1, 6) = -4680 / dt4s_[i];
    Hi(1, 7) = 360 / dt3s_[i];
    Hi(2, 0) = 10080 / dt5s_[i];
    Hi(2, 1) = 5400 / dt4s_[i];
    Hi(2, 2) = 1200 / dt3s_[i];
    Hi(2, 3) = 120 / dt2s_[i];
    Hi(2, 4) = -10080 / dt5s_[i];
    Hi(2, 5) = 4680 / dt4s_[i];
    Hi(2, 6) = -840 / dt3s_[i];
    Hi(2, 7) = 60 / dt2s_[i];
    Hi(3, 0) = 840 / dt4s_[i];
    Hi(3, 1) = 480 / dt3s_[i];
    Hi(3, 2) = 120 / dt2s_[i];
    Hi(3, 3) = 16 / dts_[i];
    Hi(3, 4) = -840 / dt4s_[i];
    Hi(3, 5) = 360 / dt3s_[i];
    Hi(3, 6) = -60 / dt2s_[i];
    Hi(3, 7) = 4 / dts_[i];
    Hi(4, 0) = -100800 / dt7s_[i];
    Hi(4, 1) = -50400 / dt6s_[i];
    Hi(4, 2) = -10080 / dt5s_[i];
    Hi(4, 3) = -840 / dt4s_[i];
    Hi(4, 4) = 100800 / dt7s_[i];
    Hi(4, 5) = -50400 / dt6s_[i];
    Hi(4, 6) = 10080 / dt5s_[i];
    Hi(4, 7) = -840 / dt4s_[i];
    Hi(5, 0) = 50400 / dt6s_[i];
    Hi(5, 1) = 24480 / dt5s_[i];
    Hi(5, 2) = 4680 / dt4s_[i];
    Hi(5, 3) = 360 / dt3s_[i];
    Hi(5, 4) = -50400 / dt6s_[i];
    Hi(5, 5) = 25920 / dt5s_[i];
    Hi(5, 6) = -5400 / dt4s_[i];
    Hi(5, 7) = 480 / dt3s_[i];
    Hi(6, 0) = -10080 / dt5s_[i];
    Hi(6, 1) = -4680 / dt4s_[i];
    Hi(6, 2) = -840 / dt3s_[i];
    Hi(6, 3) = -60 / dt2s_[i];
    Hi(6, 4) = 10080 / dt5s_[i];
    Hi(6, 5) = -5400 / dt4s_[i];
    Hi(6, 6) = 1200 / dt3s_[i];
    Hi(6, 7) = -120 / dt2s_[i];
    Hi(7, 0) = 840 / dt4s_[i];
    Hi(7, 1) = 360 / dt3s_[i];
    Hi(7, 2) = 60 / dt2s_[i];
    Hi(7, 3) = 4 / dts_[i];
    Hi(7, 4) = -840 / dt4s_[i];
    Hi(7, 5) = 480 / dt3s_[i];
    Hi(7, 6) = -120 / dt2s_[i];
    Hi(7, 7) = 16 / dts_[i];
    return Hi;
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder>
PolyTrajOpt<T, Dim, EnergyOrder>::getHiPartialGradByDti(size_t i) const
{
  if constexpr (EnergyOrder == 2) {
    Eigen::Matrix<T, 4, 4> dHi_dti;
    dHi_dti(0, 0) = -36 / dt4s_[i];
    dHi_dti(0, 1) = -12 / dt3s_[i];
    dHi_dti(0, 2) = 36 / dt4s_[i];
    dHi_dti(0, 3) = -12 / dt3s_[i];
    dHi_dti(1, 0) = -12 / dt3s_[i];
    dHi_dti(1, 1) = -4 / dt2s_[i];
    dHi_dti(1, 2) = 12 / dt3s_[i];
    dHi_dti(1, 3) = -2 / dt2s_[i];
    dHi_dti(2, 0) = 36 / dt4s_[i];
    dHi_dti(2, 1) = 12 / dt3s_[i];
    dHi_dti(2, 2) = -36 / dt4s_[i];
    dHi_dti(2, 3) = 12 / dt3s_[i];
    dHi_dti(3, 0) = -12 / dt3s_[i];
    dHi_dti(3, 1) = -2 / dt2s_[i];
    dHi_dti(3, 2) = 12 / dt3s_[i];
    dHi_dti(3, 3) = -4 / dt2s_[i];
    return dHi_dti;
  } else if constexpr (EnergyOrder == 3) {
    Eigen::Matrix<T, 6, 6> dHi_dti;
    dHi_dti(0, 0) = -3600 / dt6s_[i];
    dHi_dti(0, 1) = -1440 / dt5s_[i];
    dHi_dti(0, 2) = -180 / dt4s_[i];
    dHi_dti(0, 3) = 3600 / dt6s_[i];
    dHi_dti(0, 4) = -1440 / dt5s_[i];
    dHi_dti(0, 5) = 180 / dt4s_[i];
    dHi_dti(1, 0) = -1440 / dt5s_[i];
    dHi_dti(1, 1) = -576 / dt4s_[i];
    dHi_dti(1, 2) = -72 / dt3s_[i];
    dHi_dti(1, 3) = 1440 / dt5s_[i];
    dHi_dti(1, 4) = -504 / dt4s_[i];
    dHi_dti(1, 5) = 48 / dt3s_[i];
    dHi_dti(2, 0) = -180 / dt4s_[i];
    dHi_dti(2, 1) = -72 / dt3s_[i];
    dHi_dti(2, 2) = -9 / dt2s_[i];
    dHi_dti(2, 3) = 180 / dt4s_[i];
    dHi_dti(2, 4) = -48 / dt3s_[i];
    dHi_dti(2, 5) = 3 / dt2s_[i];
    dHi_dti(3, 0) = 3600 / dt6s_[i];
    dHi_dti(3, 1) = 1440 / dt5s_[i];
    dHi_dti(3, 2) = 180 / dt4s_[i];
    dHi_dti(3, 3) = -3600 / dt6s_[i];
    dHi_dti(3, 4) = 1440 / dt5s_[i];
    dHi_dti(3, 5) = -180 / dt4s_[i];
    dHi_dti(4, 0) = -1440 / dt5s_[i];
    dHi_dti(4, 1) = -504 / dt4s_[i];
    dHi_dti(4, 2) = -48 / dt3s_[i];
    dHi_dti(4, 3) = 1440 / dt5s_[i];
    dHi_dti(4, 4) = -576 / dt4s_[i];
    dHi_dti(4, 5) = 72 / dt3s_[i];
    dHi_dti(5, 0) = 180 / dt4s_[i];
    dHi_dti(5, 1) = 48 / dt3s_[i];
    dHi_dti(5, 2) = 3 / dt2s_[i];
    dHi_dti(5, 3) = -180 / dt4s_[i];
    dHi_dti(5, 4) = 72 / dt3s_[i];
    dHi_dti(5, 5) = -9 / dt2s_[i];
    return dHi_dti;
  } else {
    Eigen::Matrix<T, 8, 8> dHi_dti;
    dHi_dti(0, 0) = -705600 / dt8s_[i];
    dHi_dti(0, 1) = -302400 / dt7s_[i];
    dHi_dti(0, 2) = -50400 / dt6s_[i];
    dHi_dti(0, 3) = -3360 / dt5s_[i];
    dHi_dti(0, 4) = 705600 / dt8s_[i];
    dHi_dti(0, 5) = -302400 / dt7s_[i];
    dHi_dti(0, 6) = 50400 / dt6s_[i];
    dHi_dti(0, 7) = -3360 / dt5s_[i];
    dHi_dti(1, 0) = -302400 / dt7s_[i];
    dHi_dti(1, 1) = -129600 / dt6s_[i];
    dHi_dti(1, 2) = -21600 / dt5s_[i];
    dHi_dti(1, 3) = -1440 / dt4s_[i];
    dHi_dti(1, 4) = 302400 / dt7s_[i];
    dHi_dti(1, 5) = -122400 / dt6s_[i];
    dHi_dti(1, 6) = 18720 / dt5s_[i];
    dHi_dti(1, 7) = -1080 / dt4s_[i];
    dHi_dti(2, 0) = -50400 / dt6s_[i];
    dHi_dti(2, 1) = -21600 / dt5s_[i];
    dHi_dti(2, 2) = -3600 / dt4s_[i];
    dHi_dti(2, 3) = -240 / dt3s_[i];
    dHi_dti(2, 4) = 50400 / dt6s_[i];
    dHi_dti(2, 5) = -18720 / dt5s_[i];
    dHi_dti(2, 6) = 2520 / dt4s_[i];
    dHi_dti(2, 7) = -120 / dt3s_[i];
    dHi_dti(3, 0) = -3360 / dt5s_[i];
    dHi_dti(3, 1) = -1440 / dt4s_[i];
    dHi_dti(3, 2) = -240 / dt3s_[i];
    dHi_dti(3, 3) = -16 / dt2s_[i];
    dHi_dti(3, 4) = 3360 / dt5s_[i];
    dHi_dti(3, 5) = -1080 / dt4s_[i];
    dHi_dti(3, 6) = 120 / dt3s_[i];
    dHi_dti(3, 7) = -4 / dt2s_[i];
    dHi_dti(4, 0) = 705600 / dt8s_[i];
    dHi_dti(4, 1) = 302400 / dt7s_[i];
    dHi_dti(4, 2) = 50400 / dt6s_[i];
    dHi_dti(4, 3) = 3360 / dt5s_[i];
    dHi_dti(4, 4) = -705600 / dt8s_[i];
    dHi_dti(4, 5) = 302400 / dt7s_[i];
    dHi_dti(4, 6) = -50400 / dt6s_[i];
    dHi_dti(4, 7) = 3360 / dt5s_[i];
    dHi_dti(5, 0) = -302400 / dt7s_[i];
    dHi_dti(5, 1) = -122400 / dt6s_[i];
    dHi_dti(5, 2) = -18720 / dt5s_[i];
    dHi_dti(5, 3) = -1080 / dt4s_[i];
    dHi_dti(5, 4) = 302400 / dt7s_[i];
    dHi_dti(5, 5) = -129600 / dt6s_[i];
    dHi_dti(5, 6) = 21600 / dt5s_[i];
    dHi_dti(5, 7) = -1440 / dt4s_[i];
    dHi_dti(6, 0) = 50400 / dt6s_[i];
    dHi_dti(6, 1) = 18720 / dt5s_[i];
    dHi_dti(6, 2) = 2520 / dt4s_[i];
    dHi_dti(6, 3) = 120 / dt3s_[i];
    dHi_dti(6, 4) = -50400 / dt6s_[i];
    dHi_dti(6, 5) = 21600 / dt5s_[i];
    dHi_dti(6, 6) = -3600 / dt4s_[i];
    dHi_dti(6, 7) = 240 / dt3s_[i];
    dHi_dti(7, 0) = -3360 / dt5s_[i];
    dHi_dti(7, 1) = -1080 / dt4s_[i];
    dHi_dti(7, 2) = -120 / dt3s_[i];
    dHi_dti(7, 3) = -4 / dt2s_[i];
    dHi_dti(7, 4) = 3360 / dt5s_[i];
    dHi_dti(7, 5) = -1440 / dt4s_[i];
    dHi_dti(7, 6) = 240 / dt3s_[i];
    dHi_dti(7, 7) = -16 / dt2s_[i];
    return dHi_dti;
  }
}

template <typename T, size_t Dim, size_t EnergyOrder>
Eigen::SparseMatrix<T> PolyTrajOpt<T, Dim, EnergyOrder>::getCT(void) const
{
  Eigen::SparseMatrix<T> C_T(2 * _R * M_, _R * (M_ + 1));
  size_t rs = 2 * _R * M_ - _R;
  for (size_t i = 0; i < _R; ++i) {
    C_T.insert(i, i) = 1;
    C_T.insert(rs + i, _R + i) = 1;
  }
  for (size_t i = 0; i < M_ - 1; ++i) {
    for (size_t j = 0; j < _R; ++j) {
      C_T.insert(_R + 2 * i * _R + j, 2 * _R + i + j * (M_ - 1)) = 1;
      C_T.insert(_R + (2 * i + 1) * _R + j, 2 * _R + i + j * (M_ - 1)) = 1;
    }
  }
  return C_T;
}

template <typename T, size_t Dim, size_t EnergyOrder>
MatrixX<T> PolyTrajOpt<T, Dim, EnergyOrder>::getH(void) const
{
  MatrixX<T> H = MatrixX<T>::Zero(2 * M_ * _R, 2 * M_ * _R);
  for (size_t i = 0; i < M_; ++i) {
    H.block(2 * i * _R, 2 * i * _R, 2 * _R, 2 * _R) = getHi(i);
  }
  return H;
}

template <typename T, size_t Dim, size_t EnergyOrder>
MatrixX<T> PolyTrajOpt<T, Dim, EnergyOrder>::getRPartalGradByDti(size_t i) const
{
  MatrixX<T> Ci_T = C_T_.block(2 * i * _R, 0, 2 * _R, _R * M_ + _R);

  MatrixX<T> dR_dti = Ci_T.transpose() * getHiPartialGradByDti(i) * Ci_T;
  return dR_dti;
}

template <typename T, size_t Dim, size_t EnergyOrder>
VectorX<T> PolyTrajOpt<T, Dim, EnergyOrder>::getEnergyPartialGradByDt(
    const MatrixX<T>& R_PP_inv_R_PF, const MatrixX<T>& Df) const
{
  const size_t df_dim = 2 * _R + M_ - 1;
  const size_t dp_dim = (M_ - 1) * (_R - 1);
  VectorX<T> grad = VectorX<T>::Zero(M_);
  MatrixX<T> dR_dti(df_dim + dp_dim, df_dim + dp_dim);
  MatrixX<T> dR_FF_dti(df_dim, df_dim);
  MatrixX<T> dR_PP_dti(dp_dim, dp_dim);
  MatrixX<T> dR_FP_dti(df_dim, df_dim);
  MatrixX<T> G(df_dim, df_dim);
  MatrixX<T> R_PP_inv_R_PF_T = R_PP_inv_R_PF.transpose();
  MatrixX<T> dG(df_dim, df_dim);
  for (size_t i = 0; i < M_; ++i) {
    dR_dti = getRPartalGradByDti(i);
    dR_FF_dti = dR_dti.block(0, 0, df_dim, df_dim);
    G = dR_FF_dti;
    if (dp_dim != 0) {
      dR_PP_dti = dR_dti.block(df_dim, df_dim, dp_dim, dp_dim);
      dR_FP_dti = dR_dti.block(0, df_dim, df_dim, dp_dim);
      dG = dR_FP_dti * R_PP_inv_R_PF;
      G -= dG + dG.transpose();
      G += R_PP_inv_R_PF_T * dR_PP_dti * R_PP_inv_R_PF;
    }
    for (size_t j = 0; j < Dim; ++j) {
      grad[i] += Df.col(j).transpose() * G * Df.col(j);
    }
  }
  return grad;
}

#ifdef HAS_NLOPT
template <size_t Dim, size_t EnergyOrder>
void PolyTrajOptWithTimeOpt<Dim, EnergyOrder>::optimizeWithTimeOpt(
    const State& x0, const State& xf, const Waypoints& xi, double t0,
    std::vector<double>& dts, PolyTraj<double>& traj, double* J)
{
  opt_ = nlopt::opt(nlopt::LD_LBFGS, dts.size());
  if (params_.xtol_rel > 0) {
    opt_.set_xtol_rel(params_.xtol_rel);
  }
  if (params_.xtol_abs > 0) {
    opt_.set_xtol_abs(params_.xtol_abs);
  }
  if (params_.ftol_rel > 0) {
    opt_.set_ftol_rel(params_.ftol_rel);
  }
  if (params_.ftol_abs > 0) {
    opt_.set_ftol_abs(params_.ftol_abs);
  }
  if (params_.maxeval > 0) {
    opt_.set_maxeval(params_.maxeval);
  }
  if (params_.maxtime > 0) {
    opt_.set_maxtime(params_.maxtime);
  }
  std::vector<double> lb;
  for (const auto& dt : dts) {
    lb.push_back(dt * 0.1);
  }
  opt_.set_lower_bounds(lb);

  x0_ = &x0;
  xf_ = &xf;
  xi_ = &xi;
  t0_ = t0;
  t_total_ = 0;
  for (const auto& dt : dts) {
    t_total_ += dt;
  }
  traj_ = &traj;

  opt_.set_min_objective(PolyTrajOptWithTimeOpt::objectiveFunc, this);
  opt_.set_initial_step(0.1);
  double J_min;
  nlopt::result res = opt_.optimize(dts, J_min);
  if (J) {
    *J = J_min;
  }
}

template <size_t Dim, size_t EnergyOrder>
PolyTrajOptWithTimeOpt<Dim, EnergyOrder>::~PolyTrajOptWithTimeOpt(void)
{
}

template <size_t Dim, size_t EnergyOrder>
double PolyTrajOptWithTimeOpt<Dim, EnergyOrder>::objectiveFunc(
    const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  auto* traj_opt = static_cast<PolyTrajOptWithTimeOpt*>(data);
  double J;

  const auto& x0 = *traj_opt->x0_;
  const auto& xf = *traj_opt->xf_;
  const auto& xi = *traj_opt->xi_;
  const auto& t0 = traj_opt->t0_;
  const auto& t_total_ = traj_opt->t_total_;
  const auto& w_t = traj_opt->params_.weight_t;
  auto& traj = *traj_opt->traj_;
  double t_total = 0;
  for (const auto& dt : x) {
    t_total += dt;
  }

  if (!grad.empty()) {
    Eigen::Map<Eigen::VectorXd> grad_vec(grad.data(), grad.size());
    Eigen::VectorXd _grad;
    traj_opt->optimize(x0, xf, xi, t0, x, traj, &J, &_grad);
    grad_vec = _grad - _grad.mean() * Eigen::VectorXd::Ones(_grad.size());
  } else {
    traj_opt->optimize(x0, xf, xi, t0, x, traj, &J);
  }

  const double viola_t = std::abs(t_total - t_total_);
  double viola_t_pena = 0;
  double dvoila_t_pena = 0;
  if (SmoothedL1(PolyTrajOptWithTimeOpt::kSmoothMu, viola_t, viola_t_pena,
                 dvoila_t_pena)) {
    J += w_t * viola_t_pena;
    if (!grad.empty()) {
      for (size_t i = 0; i < x.size(); ++i) {
        grad[i] += w_t * dvoila_t_pena;
      }
    }
  }

  return J;
}
#endif  // HAS_NLOPT

template <typename T>
void PolyTrajSegUniAcc(T max_vel, T max_acc, T t0, const Vector2<T>& x0,
                       T xf, PolyTraj<T>& traj)
{
  RU_ASSERT(max_vel > 0, "Max velocity must be greater than 0");
  RU_ASSERT(max_acc > 0, "Max acceleration must be greater than 0");

  T _x0 = x0(0);
  T _v0 = std::clamp(x0(1), -max_vel, max_vel);
  T dir = GetSign(xf - _x0);
  const T dist_thresh = (2 * max_vel * max_vel - _v0 * _v0) / (2 * max_acc);

  if (std::abs(xf - _x0) <= dist_thresh) {
    // Triangular segment
    std::vector<MatrixX<T>> coeffs;
    std::vector<T> dts;
    T v1 = dir * std::sqrt(max_acc * std::abs(xf - _x0) + _v0 * _v0 / 2);
    while (std::abs(v1) < dir * _v0) {
      T _v0_new = -GetSign(_v0) *
                  std::sqrt(max_acc * std::abs(xf - _x0) + _v0 * _v0 / 2);
      T dt = std::abs(_v0_new - _v0) / max_acc;
      MatrixX<T> coeff(1, 3);
      coeff(0, 0) = _x0;
      coeff(0, 1) = _v0;
      coeff(0, 2) = -dir * max_acc / 2;
      coeffs.push_back(coeff);
      dts.push_back(dt);
      _x0 += (_v0 + _v0_new) * dt / 2;
      _v0 = _v0_new;
      dir = GetSign(xf - _x0);
      v1 = dir * std::sqrt(max_acc * std::abs(xf - _x0) + _v0 * _v0 / 2);
    }
    T t1 = std::abs(v1 - _v0) / max_acc;
    T x1 = (_v0 + v1) * t1 / 2 + _x0;
    T t2 = std::abs(v1) / max_acc;
    MatrixX<T> coeff1(1, 3);
    coeff1(0, 0) = _x0;
    coeff1(0, 1) = _v0;
    coeff1(0, 2) = dir * max_acc / 2;
    coeffs.push_back(coeff1);
    dts.push_back(t1);
    MatrixX<T> coeff2(1, 3);
    coeff2(0, 0) = x1;
    coeff2(0, 1) = v1;
    coeff2(0, 2) = -coeff1(0, 2);
    coeffs.push_back(coeff2);
    dts.push_back(t2);
    traj.setTraj(t0, coeffs, dts, false);
  } else {
    // Trapezoidal segment
    T v1 = dir * max_vel;
    T t1 = std::abs(v1 - _v0) / max_acc;
    T x1 = (_v0 + v1) * t1 / 2 + _x0;
    T t2 = (std::abs(xf - _x0) - dist_thresh) / max_vel;
    T x2 = x1 + v1 * t2;
    T t3 = max_vel / max_acc;
    MatrixX<T> coeff1(1, 3);
    coeff1(0, 0) = _x0;
    coeff1(0, 1) = _v0;
    coeff1(0, 2) = dir * max_acc / 2;
    MatrixX<T> coeff2(1, 2);
    coeff2(0, 0) = x1;
    coeff2(0, 1) = v1;
    MatrixX<T> coeff3(1, 3);
    coeff3(0, 0) = x2;
    coeff3(0, 1) = v1;
    coeff3(0, 2) = -coeff1(0, 2);
    traj.setTraj(t0, {coeff1, coeff2, coeff3}, {t1, t2, t3}, false);
  }
}

template class Poly<float>;
template class Poly<double>;
template class PolyTraj<float>;
template class PolyTraj<double>;

template class PolyTrajOpt<float, 1, 2>;
template class PolyTrajOpt<double, 1, 2>;
template class PolyTrajOpt<float, 2, 2>;
template class PolyTrajOpt<double, 2, 2>;
template class PolyTrajOpt<float, 3, 2>;
template class PolyTrajOpt<double, 3, 2>;
template class PolyTrajOpt<float, 1, 3>;
template class PolyTrajOpt<double, 1, 3>;
template class PolyTrajOpt<float, 2, 3>;
template class PolyTrajOpt<double, 2, 3>;
template class PolyTrajOpt<float, 3, 3>;
template class PolyTrajOpt<double, 3, 3>;
template class PolyTrajOpt<float, 1, 4>;
template class PolyTrajOpt<double, 1, 4>;
template class PolyTrajOpt<float, 2, 4>;
template class PolyTrajOpt<double, 2, 4>;
template class PolyTrajOpt<float, 3, 4>;
template class PolyTrajOpt<double, 3, 4>;

#ifdef HAS_NLOPT
template class PolyTrajOptWithTimeOpt<1, 2>;
template class PolyTrajOptWithTimeOpt<2, 2>;
template class PolyTrajOptWithTimeOpt<3, 2>;
template class PolyTrajOptWithTimeOpt<1, 3>;
template class PolyTrajOptWithTimeOpt<2, 3>;
template class PolyTrajOptWithTimeOpt<3, 3>;
template class PolyTrajOptWithTimeOpt<1, 4>;
template class PolyTrajOptWithTimeOpt<2, 4>;
template class PolyTrajOptWithTimeOpt<3, 4>;
#endif  // HAS_NLOPT

template void PolyTrajSegUniAcc<float>(float max_vel, float max_acc, float t0,
                                       const Eigen::Vector2f& x0, float xf,
                                       PolyTraj<float>& traj);
template void PolyTrajSegUniAcc<double>(double max_vel, double max_acc,
                                        double t0, const Eigen::Vector2d& x0,
                                        double xf, PolyTraj<double>& traj);
/* Private function definitions ----------------------------------------------*/

template <typename T>
static T PolyCoeffDerivate(size_t n, size_t n_d)
{
  RU_ASSERT(n >= 0, "Polynomial order must be greater than or equal to 0");
  if (n_d > n) {
    return 0;
  }

  T coeff = 1;
  for (size_t i = 0; i < n_d; ++i) {
    coeff *= (n - i);
  }
  return coeff;
}

#ifdef HAS_NLOPT
bool SmoothedL1(double mu, double x, double& f, double& df)
{
  if (x < 0.0) {
    return false;
  } else if (x > mu) {
    f = x - 0.5 * mu;
    df = 1.0;
    return true;
  } else {
    const double mu2 = mu * mu;
    const double mu3 = mu2 * mu;
    const double c4 = -1 / (2 * mu3);
    const double c3 = 1 / mu2;
    const double x2 = x * x;
    const double x3 = x2 * x;

    f = (c3 + c4 * x) * x3;
    df = (3 * c3 + 4 * c4 * x) * x2;
    return true;
  }
}
#endif  // HAS_NLOPT
}  // namespace robot_utils
