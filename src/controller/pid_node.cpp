/**
 *******************************************************************************
 * @file      : pid_node.cpp
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
#include "robot_utils/controller/pid_node.hpp"

#include <algorithm>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/periodic_data.hpp"
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
PidNode<T>::PidNode(const Params& params) : data_(), td_ptr_(nullptr)
{
  setParams(params);

  reset();
}

template <typename T>
T PidNode<T>::calc(const T& ref, const T& fdb, const T& ffd)
{
  const T dt = 1 / params_.ctrl_rate;
  data_.prev_ref = data_.ref;
  data_.prev_fdb = data_.fdb;
  data_.prev_err = data_.err;
  data_.prev_out = data_.out;

  data_.ref = ref;
  data_.fdb = fdb;
  T err = 0;
  if (params_.period > 0) {
    err = PeriodicDataSub(params_.period, ref, fdb);
  } else {
    err = ref - fdb;
  }

  if (params_.en_deadband) {
    data_.err = std::clamp(err, params_.deadband_lb, params_.deadband_ub);
  } else {
    data_.err = err;
  }

  // Calculate the proportional term
  data_.pout = params_.kp * data_.err;

  // Calculate the integral term
  T err_i = params_.en_trap_int ? (data_.err + data_.prev_err) / 2 : data_.err;
  T anti_windup_scale = 1;
  if (params_.en_anti_windup) {
    if (data_.iout > params_.anti_windup_up && err_i > 0) {
      anti_windup_scale = 0;
    } else if (data_.iout < params_.anti_windup_lb && err_i < 0) {
      anti_windup_scale = 0;
    }
  }
  T int_separate_scale = 1;
  if (params_.en_int_separate &&
      !IsInRange(params_.int_separate_lb, params_.int_separate_ub, err_i)) {
    int_separate_scale = 0;
  }
  data_.iout +=
      params_.ki * err_i * dt * anti_windup_scale * int_separate_scale;

  // Calculate the derivative term
  T err_d = params_.perv_diff_weight * data_.fdb +
            (1 - params_.perv_diff_weight) * data_.err;
  if (params_.en_td) {
    data_.dout = params_.kd * td_ptr_->calc(err_d);
  } else {
    T prev_err_d = params_.perv_diff_weight * data_.prev_fdb +
                   (1 - params_.perv_diff_weight) * data_.prev_err;
    data_.dout = params_.kd * (err_d - prev_err_d) / dt;
  }

  if (params_.en_out_limit) {
    T out = data_.pout + data_.iout + data_.dout + ffd;
    data_.out = std::clamp(out, params_.out_limit_lb, params_.out_limit_ub);
  } else {
    data_.out = data_.pout + data_.iout + data_.dout + ffd;
  }

  return data_.out;
}

template <typename T>
void PidNode<T>::setParams(const Params& params)
{
  PARAM_ASSERT(params.kp >= 0, "kp must be greater than or equal to 0");
  PARAM_ASSERT(params.ki >= 0, "ki must be greater than or equal to 0");
  PARAM_ASSERT(params.kd >= 0, "kd must be greater than or equal to 0");
  PARAM_ASSERT(params.ctrl_rate > 0, "ctrl_rate must be greater than 0");
  PARAM_ASSERT(
      params_.en_out_limit && params_.out_limit_lb < params_.out_limit_ub,
      "out_limit_lb must be less than out_limit_ub");
  PARAM_ASSERT(params_.en_deadband && params_.deadband_lb < params_.deadband_ub,
               "deadband_lb must be less than deadband_ub");
  PARAM_ASSERT(
      params_.en_anti_windup && params_.anti_windup_lb < params_.anti_windup_up,
      "anti_windup_lb must be less than anti_windup_up");
  PARAM_ASSERT(params_.en_int_separate &&
                   params_.int_separate_lb < params_.int_separate_ub,
               "int_separate_lb must be less than int_separate_ub");
  PARAM_ASSERT(params_.perv_diff_weight >= 0 && params_.perv_diff_weight <= 1,
               "perv_diff_weight must be between 0 and 1");

  params_ = params;
  params_.td_params.period = params_.period;
  params_.td_params.dt = 1 / params_.ctrl_rate;

  if (params_.en_td) {
    if (td_ptr_ == nullptr) {
      td_ptr_ = std::make_shared<Td<T>>(params_.td_params);
    } else {
      td_ptr_->setParams(params_.td_params);
    }
  } else {
    td_ptr_ = nullptr;
  }
}

template class PidNode<float>;
template class PidNode<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils