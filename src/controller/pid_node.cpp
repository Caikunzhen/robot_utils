/**
 *******************************************************************************
 * @file pid_node.cpp
 * @brief Basic PID controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-08
 * @author Caikunzhen
 * @details
 * 1. Complete the pid_node.cpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/controller/pid_node.hpp"

#include <algorithm>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/math_tools.hpp"
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
PidNode<T>::PidNode(const Params& params)
{
  RU_ASSERT(params.dt > 0, "dt must be greater than 0");

  params_.dt = params.dt;
  params_.period = params.period;
  setParams(params);

  reset();
}

template <typename T>
T PidNode<T>::calc(const T& ref, const T& fdb, const T& ffd)
{
  const T& dt = params_.dt;
  data_.prev_ref = data_.ref;
  data_.prev_fdb = data_.fdb;
  data_.prev_err = data_.err;
  data_.prev_out = data_.out;

  // Calculate the error
  data_.ref = ref;
  data_.fdb = fdb;
  T err = 0;
  if (params_.period > 0) {
    err = PeriodicDataSub(params_.period, ref, fdb);
  } else {
    err = ref - fdb;
  }

  if (params_.en_deadband) {
    data_.err = Deadband(params_.deadband_lb, params_.deadband_ub, err);
  } else {
    data_.err = err;
  }

  // Calculate the proportional term
  data_.pout = params_.kp * data_.err;

  // Calculate the integral term
  T err_i = params_.en_trap_int ? (data_.err + data_.prev_err) / 2 : data_.err;
  if (params_.en_int_separate &&
      !IsInRange(params_.int_separate_lb, params_.int_separate_ub, err_i)) {
    err_i = 0;
  }
  if (params_.en_anti_windup) {
    data_.iout = std::clamp(data_.iout + params_.ki * err_i * dt,
                            params_.anti_windup_lb, params_.anti_windup_ub);
  } else {
    data_.iout += params_.ki * err_i * dt;
  }

  // Calculate the derivative term
  T derr = 0, dfdb = 0;
  if (params_.en_td) {
    derr = err_td_ptr_->calc(data_.err);
    dfdb = fdb_td_ptr_->calc(data_.fdb);
  } else if (!data_.is_first_calc) {
    derr = (data_.err - data_.prev_err) / dt;
    if (params_.period > 0) {
      dfdb = PeriodicDataSub(params_.period, data_.fdb, data_.prev_fdb) / dt;
    } else {
      dfdb = (data_.fdb - data_.prev_fdb) / dt;
    }
  }
  data_.dout = params_.kd * (-params_.perv_diff_weight * dfdb +
                             (1 - params_.perv_diff_weight) * derr);

  // Calculate the output
  T out = data_.pout + data_.iout + data_.dout + ffd;
  if (params_.en_out_limit) {
    data_.out = std::clamp(out, params_.out_limit_lb, params_.out_limit_ub);
  } else {
    data_.out = out;
  }

  data_.is_first_calc = false;

  return data_.out;
}

template <typename T>
void PidNode<T>::setParams(const Params& params)
{
  RU_ASSERT(params.kp >= 0, "kp must be greater than or equal to 0");
  RU_ASSERT(params.ki >= 0, "ki must be greater than or equal to 0");
  RU_ASSERT(params.kd >= 0, "kd must be greater than or equal to 0");
  RU_ASSERT(
      !params_.en_out_limit || params_.out_limit_lb < params_.out_limit_ub,
      "out_limit_lb must be less than out_limit_ub");
  RU_ASSERT(!params_.en_deadband || params_.deadband_lb < params_.deadband_ub,
            "deadband_lb must be less than deadband_ub");
  RU_ASSERT(!params_.en_anti_windup ||
                params_.anti_windup_lb < params_.anti_windup_ub,
            "anti_windup_lb must be less than anti_windup_ub");
  RU_ASSERT(!params_.en_int_separate ||
                params_.int_separate_lb < params_.int_separate_ub,
            "int_separate_lb must be less than int_separate_ub");
  RU_ASSERT(params_.perv_diff_weight >= 0 && params_.perv_diff_weight <= 1,
            "perv_diff_weight must be between 0 and 1");

  T dt = params_.dt;
  T period = params_.period;
  params_ = params;
  params_.dt = dt;
  params_.period = period;

  if (params_.en_td) {
    TdParams<T> err_td_params{
        .cutoff_freq = params_.td_cutoff_freq,
        .dt = params_.dt,
    };
    TdParams<T> fdb_td_params{
        .cutoff_freq = params_.td_cutoff_freq,
        .dt = params_.dt,
        .period = params_.period,
    };
    if (err_td_ptr_ == nullptr) {
      err_td_ptr_ = std::make_shared<Td<T>>(err_td_params);
      fdb_td_ptr_ = std::make_shared<Td<T>>(fdb_td_params);
    } else {
      err_td_ptr_->setParams(err_td_params);
      fdb_td_ptr_->setParams(fdb_td_params);
    }
  } else {
    err_td_ptr_ = nullptr;
    fdb_td_ptr_ = nullptr;
  }
}

template class PidNode<float>;
template class PidNode<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils