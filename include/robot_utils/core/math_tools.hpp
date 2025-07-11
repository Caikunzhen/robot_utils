/**
 *******************************************************************************
 * @file math_tools.hpp
 * @brief This file provides the math tools for robot_utils.
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-04-30
 * @author Caikunzhen
 * @details
 * 1. Complete the math_tools.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CORE_MATH_TOOLS_HPP_
#define ROBOT_UTILS_CORE_MATH_TOOLS_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <type_traits>
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/

/// Gravity acceleration(Hangzhou, China), unit: m/s^2
static constexpr double kGravAcc = 9.7936;
static constexpr double kRad2DegCoff = 180.0 / M_PI;
static constexpr double kDeg2RadCoff = M_PI / 180.0;
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

inline float Deg2Rad(float deg)
{
  return deg * static_cast<float>(kDeg2RadCoff);
}

inline double Deg2Rad(double deg) { return deg * kDeg2RadCoff; }

inline float Rad2Deg(float rad)
{
  return rad * static_cast<float>(kRad2DegCoff);
}

inline double Rad2Deg(double rad) { return rad * kRad2DegCoff; }

/**
 * @brief Generic mod function
 *
 * This function performs a modulus operation for any arithmetic type.
 * For integral types, it uses the % operator.
 * For floating-point types, it uses the std::fmod function.
 *
 * @tparam T Arithmetic type (integer or floating point)
 * @param[in] val: The value to be modded
 * @param[in] mod: The modulus
 * @return The result of the modulus operation(when mod = 0, return val)
 */
template <typename T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
inline T GenericMod(const T& val, const T& mod)
{
  if (mod == 0) {
    return val;
  }
  if constexpr (std::is_integral_v<T>) {
    return val % mod;
  } else {
    T result = std::fmod(val, mod);
    return result;
  }
}

/**
 * @brief Generic sign function
 *
 * This function returns the sign of a value.
 *
 * @tparam T Type of the value
 * @param[in] val: The value to check
 * @return 1 if val > 0, -1 if val < 0, and 0 if val == 0
 */
template <typename T>
inline T GetSign(const T& val)
{
  if (val > 0) {
    return 1;
  } else if (val == 0) {
    return 0;
  } else {
    return -1;
  }
}

/**
 * @brief Check if a value is in a specified range
 * @tparam T Type of the value
 * @param[in] min_val: Minimum value of the range
 * @param[in] max_val: Maximum value of the range
 * @param[in] val: Value to check
 * @return true if val is in the range [min_val, max_val], false otherwise
 */
template <typename T>
inline bool IsInRange(const T& min_val, const T& max_val, const T& val)
{
  return val >= min_val && val <= max_val;
}

/**
 * @brief Deadband function
 * @tparam T Type of the value
 * @param[in] lb: Lower bound of the deadband
 * @param[in] ub: Upper bound of the deadband
 * @param[in] val: Value to check
 * @return 0 if val is in the deadband [lb, ub], otherwise return val
 */
template <typename T>
inline T Deadband(const T& lb, const T& ub, const T& val)
{
  if (lb <= val && val <= ub) {
    return 0;
  } else {
    return val;
  }
}

/**
 * @brief Deadband function
 * @tparam T Type of the value
 * @param[in] lb: Lower bound of the deadband
 * @param[in] ub: Upper bound of the deadband
 * @param[in,out] val: Value to check and modify, it will be set to 0 if it is
 * in the deadband [lb, ub]
 */
template <typename T>
inline void Deadband(const T& lb, const T& ub, T* val)
{
  *val = Deadband(lb, ub, *val);
}
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CORE_MATH_TOOLS_HPP_ */
