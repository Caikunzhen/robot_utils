/**
 *******************************************************************************
 * @file time.hpp
 * @brief This file provides the time function for robot_utils.
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-04-30
 * @author Caikunzhen
 * @details
 * 1. Complete the time.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CORE_TIME_HPP_
#define ROBOT_UTILS_CORE_TIME_HPP_

/* Includes ------------------------------------------------------------------*/
#include <chrono>
#include <type_traits>
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Get current time in seconds
 *
 * This function returns the current time in seconds.
 *
 * @tparam T Type of the time, should be floating point
 * @return The current time in seconds
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T GetCurrTime(void)
{
  auto now = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch());
  return static_cast<T>(now.count()) * static_cast<T>(1e-9);
}
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CORE_TIME_HPP_ */
