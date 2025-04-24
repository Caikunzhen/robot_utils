/**
 *******************************************************************************
 * @file      : periodic_data.hpp
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CORE_PERIODIC_DATA_HPP_
#define ROBOT_UTILS_CORE_PERIODIC_DATA_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cmath>
#include <type_traits>

#include "robot_utils/core/math_tools.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Periodic data normalization
 *
 * This function normalizes the data to [lb, lb + period) range.
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] lb The lower bound of the data
 * @param[in] data The data to be normalized
 * @return The normalized data
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline T PeriodicDataNorm(const T& period, const T& lb, const T& data)
{
  T data_norm = GenericMod(data - lb, period);
  if (data_norm < 0) {
    data_norm += period;
  }
  return data_norm + lb;
}

/**
 * @brief Periodic data normalization
 *
 * This function normalizes the data to [lb, lb + period) range.
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] lb The lower bound of the data
 * @param[in|out] data The data to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline void PeriodicDataNorm(const T& period, const T& lb, T* data)
{
  *data = PeriodicDataNorm(period, lb, *data);
}

/**
 * @brief Periodic data normalization
 *
 * This function normalizes the data to [-period/2, period/2) range.
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] data The data to be normalized
 * @return The normalized data
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline T PeriodicDataNorm(const T& period, const T& data)
{
  return PeriodicDataNorm(period, -period / 2, data);
}

/**
 * @brief Periodic data normalization
 *
 * This function normalizes the data to [-period/2, period/2) range.
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in|out] data The data to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline void PeriodicDataNorm(const T& period, T* data)
{
  *data = PeriodicDataNorm(period, *data);
}

/**
 * @brief Periodic data subtraction
 *
 * This function computes the subtraction of the reference value and the data,
 * and normalizes the result to [-period/2, period/2) range.
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] minuend The reference value
 * @param[in] subtrahend The data to be subtracted
 * @return The result of the subtraction
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline T PeriodicDataSub(const T& period, const T& minuend, const T& subtrahend)
{
  return PeriodicDataNorm(period, minuend - subtrahend);
}

/**
 * @brief Transform periodic data to minimum distance from the reference
 *
 * This function transforms the data to the minimum distance from the reference
 * value, the distance is in the range of [-period/2, period/2).
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] ref The reference value
 * @param[in] data The data to be transformed
 * @return The transformed data
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline T PeriodicData2MinDist(const T& period, const T& ref, const T& data)
{
  return PeriodicDataNorm(period, ref - period / 2, data);
}

/**
 * @brief Transform periodic data to minimum distance from the reference
 *
 * This function transforms the data to the minimum distance from the reference
 * value, the distance is in the range of [-period/2, period/2).
 *
 * @tparam T Type of the data, should be signed
 * @param[in] period The period of the data, should be positive
 * @param[in] ref The reference value
 * @param[in|out] data The data to be transformed, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_signed_v<T>>>
inline void PeriodicData2MinDist(const T& period, const T& ref, T* data)
{
  *data = PeriodicData2MinDist(period, ref, *data);
}

/**
 * @brief Angle normalization(radians)
 *
 * This function normalizes the angle to [lb, lb + 2 * pi) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] lb The lower bound of the angle
 * @param[in] angle The angle to be normalized
 * @return The normalized angle
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleRadNorm(const T& lb, const T& angle)
{
  return PeriodicDataNorm(static_cast<T>(2 * M_PI), lb, angle);
}

/**
 * @brief Angle normalization(radians)
 *
 * This function normalizes the angle to [lb, lb + 2 * pi) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] lb The lower bound of the angle
 * @param[in|out] angle The angle to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleRadNorm(const T& lb, T* angle)
{
  *angle = AngleRadNorm(lb, *angle);
}

/**
 * @brief Angle normalization(radians)
 *
 * This function normalizes the angle to [-pi, pi) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] angle The angle to be normalized
 * @return The normalized angle
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleRadNorm(const T& angle)
{
  return AngleRadNorm(static_cast<T>(-M_PI), angle);
}

/**
 * @brief Angle normalization(radians)
 *
 * This function normalizes the angle to [-pi, pi) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in|out] angle The angle to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleRadNorm(T* angle)
{
  *angle = AngleRadNorm(*angle);
}

/**
 * @brief Angle subtraction(radians)
 *
 * This function computes the subtraction of the reference value and the angle,
 * and normalizes the result to [-pi, pi) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] minuend The reference value
 * @param[in] subtrahend The angle to be subtracted
 * @return The result of the subtraction
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleRadSub(const T& minuend, const T& subtrahend)
{
  return PeriodicDataSub(static_cast<T>(2 * M_PI), minuend, subtrahend);
}

/**
 * @brief Transform angle to minimum distance from the reference(radians)
 *
 * This function transforms angle to the minimum distance from the reference
 * value, the distance is in the range of [-pi, pi).
 *
 * @tparam T Type of the data, should be floating point
 * @param[in] ref The reference value
 * @param[in] data The data to be transformed
 * @return The transformed data
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleRad2MinDist(const T& ref, const T& data)
{
  return PeriodicData2MinDist(static_cast<T>(2 * M_PI), ref, data);
}

/**
 * @brief Transform angle to minimum distance from the reference(radians)
 *
 * This function transforms angle to the minimum distance from the reference
 * value, the distance is in the range of [-pi, pi).
 *
 * @tparam T Type of the data, should be floating point
 * @param[in] ref The reference value
 * @param[in|out] data The data to be transformed, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleRad2MinDist(const T& ref, T* data)
{
  *data = AngleRad2MinDist(ref, *data);
}

/**
 * @brief Angle normalization(degrees)
 *
 * This function normalizes the angle to [lb, lb + 360) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] lb The lower bound of the angle
 * @param[in] angle The angle to be normalized
 * @return The normalized angle
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleDegNorm(const T& lb, const T& angle)
{
  return PeriodicDataNorm(static_cast<T>(360), lb, angle);
}

/**
 * @brief Angle normalization(degrees)
 *
 * This function normalizes the angle to [lb, lb + 360) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] lb The lower bound of the angle
 * @param[in|out] angle The angle to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleDegNorm(const T& lb, T* angle)
{
  *angle = AngleRadNorm(lb, *angle);
}

/**
 * @brief Angle normalization(degrees)
 *
 * This function normalizes the angle to [-180, 180) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] angle The angle to be normalized
 * @return The normalized angle
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleDegNorm(const T& angle)
{
  return AngleDegNorm(static_cast<T>(-180), angle);
}

/**
 * @brief Angle normalization(degrees)
 *
 * This function normalizes the angle to [-180, 180) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in|out] angle The angle to be normalized, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleDegNorm(T* angle)
{
  *angle = AngleDegNorm(*angle);
}

/**
 * @brief Angle subtraction(degrees)
 *
 * This function computes the subtraction of the reference value and the angle,
 * and normalizes the result to [-180, 180) range.
 *
 * @tparam T Type of the angle, should be floating point
 * @param[in] minuend The reference value
 * @param[in] subtrahend The angle to be subtracted
 * @return The result of the subtraction
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleDegSub(const T& minuend, const T& subtrahend)
{
  return PeriodicDataSub(static_cast<T>(360), minuend, subtrahend);
}

/**
 * @brief Transform angle to minimum distance from the reference(degrees)
 *
 * This function transforms angle to the minimum distance from the reference
 * value, the distance is in the range of [-180, 180).
 *
 * @tparam T Type of the data, should be floating point
 * @param[in] ref The reference value
 * @param[in] data The data to be transformed
 * @return The transformed data
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline T AngleDeg2MinDist(const T& ref, const T& data)
{
  return PeriodicData2MinDist(static_cast<T>(360), ref, data);
}

/**
 * @brief Transform angle to minimum distance from the reference(degrees)
 *
 * This function transforms angle to the minimum distance from the reference
 * value, the distance is in the range of [-180, 180).
 *
 * @tparam T Type of the data, should be floating point
 * @param[in] ref The reference value
 * @param[in|out] data The data to be transformed, which is modified in place
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleDeg2MinDist(const T& ref, T* data)
{
  *data = AngleDeg2MinDist(ref, *data);
}
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CORE_PERIODIC_DATA_HPP_ */
