/**
 *******************************************************************************
 * @file lqr.cpp
 * @brief Linear Quadratic Regulator (LQR) controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-10
 * @author Caikunzhen
 * @details
 * 1. Complete the lqr.cpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/controller/lqr.hpp"

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/time.hpp"
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

template <typename T>
Lqr<T>::Lqr(const Params& params)
{
  RU_ASSERT(params.n > 0, "n must be greater than 0");
  RU_ASSERT(params.m > 0, "m must be greater than 0");

  params_.n = params.n;
  params_.m = params.m;
  setParams(params);
}

template <typename T>
bool Lqr<T>::solve(void)
{
  if (data_.is_converged) {
    return true;
  }
  T start_time = GetCurrTime<T>();

  const auto& A = params_.A;
  const auto A_T = A.transpose();
  const auto& B = params_.B;
  const auto B_T = B.transpose();
  const auto Q = params_.Q.toDenseMatrix();
  const auto R = params_.R.toDenseMatrix();
  auto& K = data_.K;
  auto& P = data_.P;

  data_.is_converged = false;
  size_t i = 0;
  T cost_time = 0;
  while (1) {
    K = (R + B_T * P * B).ldlt().solve(B_T * P * A);
    MatrixX<T> A_cl = A - B * K;
    MatrixX<T> Res = A_T * P * A_cl + Q - P;

    data_.res = Res.cwiseAbs().maxCoeff();
    cost_time = GetCurrTime<T>() - start_time;
    if (data_.res < params_.tol) {
      data_.is_converged = true;
      break;
    }

    if ((params_.max_iter > 0 && i >= params_.max_iter) ||
        (params_.max_cost_time > 0 && cost_time > params_.max_cost_time) ||
        i >= 1e8) {
      break;
    }

    MatrixX<T> DeltaP = Res;
    MatrixX<T> A_cl_T = A_cl.transpose();
    for (size_t j = 0; j < 100; ++j) {
      DeltaP = A_cl_T * DeltaP * A_cl + Res;
    }
    P += DeltaP;
    ++i;
  }
  data_.iter = i;
  data_.cost_time = cost_time;

  return data_.is_converged;
}

template <typename T>
void Lqr<T>::calc(const StateVec& ref, const StateVec& fdb, InputVec& u) const
{
  RU_ASSERT(ref.size() == params_.n, "ref size must be equal to n");
  RU_ASSERT(fdb.size() == params_.n, "fdb size must be equal to n");
  RU_ASSERT(u.size() == params_.m, "u size must be equal to m");
  if (!data_.is_converged) {
    RU_ASSERT(false, "LQR not converged, call solve() first");
    return;
  }

  u = data_.K * (ref - fdb);
}

template <typename T>
void Lqr<T>::setParams(const Params& params)
{
  RU_ASSERT(params.max_iter > 0 || params.max_cost_time > 0,
            "max_iter or max_cost_time must be greater than 0");
  RU_ASSERT(params.tol > 0, "tol must be greater than 0");
  RU_ASSERT(params.A.rows() == params_.n && params.A.cols() == params_.n,
            "A must be a square matrix of size (n, n)");
  RU_ASSERT(params.B.rows() == params_.n && params.B.cols() == params_.m,
            "B must be a matrix of size (n, m)");
  RU_ASSERT(params.Q.rows() == params_.n && params.Q.cols() == params_.n,
            "Q must be a square matrix of size (n, n)");
  RU_ASSERT(params.R.rows() == params_.m && params.R.cols() == params_.m,
            "R must be a square matrix of size (m, m)");

  T n = params_.n;
  T m = params_.m;
  params_ = params;
  params_.n = n;
  params_.m = m;
  data_.P = params_.Q;
  data_.res = std::numeric_limits<T>::max();
  data_.is_converged = false;
}

template class Lqr<float>;
template class Lqr<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
