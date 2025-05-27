/**
 *******************************************************************************
 * @file td.cpp
 * @brief Tracking Differentiator
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-08
 * @author Caikunzhen
 * @details
 * 1. Complete the td.cpp
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
  RU_ASSERT(params.dt > 0, "dt must be greater than 0");

  params_.dt = params.dt;
  params_.period = params.period;
  setParams(params);

  reset();
}

template <typename T>
void Td<T>::setParams(const Params& params)
{
  RU_ASSERT(params.cutoff_freq > 0, "cutoff_freq must be greater than 0");

  T dt = params_.dt;
  T period = params_.period;
  params_ = params;
  params_.dt = dt;
  params_.period = period;

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
