/**
 *******************************************************************************
 * @file      : lqr.cpp
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
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/controller/lqr.hpp"

#include "robot_utils/core/assert.hpp"
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
bool Lqr<T>::solve(void)
{
  if (data_.is_converged) {
    return true;
  }

  const auto& A = params_.A;
  auto A_T = A.transpose();
  const auto& B = params_.B;
  auto B_T = B.transpose();
  const auto& Q = params_.Q;
  const auto& R = params_.R;
  auto& K = data_.K;
  auto& P = data_.P;

  data_.is_converged = false;
  for (size_t i = 0; i < params_.max_iter; ++i) {
    K = (R + B_T * P * B).llt().solve(B_T * P * A);
    Eigen::MatrixX<T> A_cl = A - B * K;
    Eigen::MatrixX<T> Res = P - A_T * P * A_cl - Q;

    data_.res = Res.cwiseAbs().maxCoeff();
    if (data_.res < params_.tol) {
      data_.is_converged = true;
      break;
    }

    Eigen::MatrixX<T> DeltaP = Res;
    Eigen::MatrixX<T> A_cl_T = A_cl.transpose();
    for (size_t j = 0; j < 10; ++j) {
      DeltaP += A_cl_T * DeltaP * A_cl + Res;
    }
    P += DeltaP;
  }

  return data_.is_converged;
}

template <typename T>
void Lqr<T>::calc(const StateVec& x, InputVec& u) const
{
  PARAM_ASSERT(x.size() == params_.n, "x size must be equal to n");
  PARAM_ASSERT(u.size() == params_.m, "u size must be equal to m");
  if (!data_.is_converged) {
    PARAM_ASSERT(false, "LQR not converged, call solve() first");
    return;
  }

  u = -data_.K * x;
}

template <typename T>
void Lqr<T>::calc(const StateVec& ref, const StateVec& fdb, InputVec& u) const
{
  PARAM_ASSERT(ref.size() == params_.n, "ref size must be equal to n");
  PARAM_ASSERT(fdb.size() == params_.n, "fdb size must be equal to n");
  PARAM_ASSERT(u.size() == params_.m, "u size must be equal to m");
  if (!data_.is_converged) {
    PARAM_ASSERT(false, "LQR not converged, call solve() first");
    return;
  }

  u = data_.K * (ref - fdb);
}

template <typename T>
void Lqr<T>::setParams(const Params& params)
{
  PARAM_ASSERT(params.n > 0, "n must be greater than 0");
  PARAM_ASSERT(params.m > 0, "m must be greater than 0");
  PARAM_ASSERT(params.max_iter > 0, "max_iter must be greater than 0");
  PARAM_ASSERT(params.tol > 0, "tol must be greater than 0");
  PARAM_ASSERT(params.A.rows() == params.n && params.A.cols() == params.n,
               "A must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.B.rows() == params.n && params.B.cols() == params.m,
               "B must be a matrix of size (n, m)");
  PARAM_ASSERT(params.Q.rows() == params.n && params.Q.cols() == params.n,
               "Q must be a square matrix of size (n, n)");
  PARAM_ASSERT(params.R.rows() == params.m && params.R.cols() == params.m,
               "R must be a square matrix of size (m, m)");

  params_ = params;
  data_.P = params_.Q;
  data_.res = std::numeric_limits<T>::max();
  data_.is_converged = false;
}

template class Lqr<float>;
template class Lqr<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
