/**
 *******************************************************************************
 * @file      : td.cpp
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
#include "robot_utils/filter/td.hpp"

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
Td<T>::Td(const Params& params)
{
  setParams(params);

  reset();
}

template <typename T>
void Td<T>::setParams(const Params& params)
{
  PARAM_ASSERT(params.cutoff_freq > 0, "cutoff_freq must be greater than 0");
  PARAM_ASSERT(params.dt > 0, "dt must be greater than 0");

  params_ = params;

  data_.is_first_calc = true;
}

template <typename T>
T Td<T>::calc(const T& x)
{
  const T& period = params_.period;
  const T& dt = params_.dt;
  const T& r = params_.cutoff_freq;

  if (data_.is_first_calc) {
    data_.is_first_calc = false;
    data_.x = x;
  }

  T diff_x = 0;
  if (params_.period > 0) {
    diff_x = PeriodicDataSub(period, data_.x, x);
    data_.x = PeriodicDataNorm(period, data_.x + dt * data_.dx);
  } else {
    diff_x = data_.x - x;
    data_.x = data_.x + dt * data_.dx;
  }

  data_.dx = data_.dx - dt * (r * r * diff_x + 2 * r * data_.dx);

  return data_.dx;
}

template class Td<float>;
template class Td<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
