/**
 *******************************************************************************
 * @file core.hpp
 * @brief This file provides the basic function for geometry process.
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-04-30
 * @author Caikunzhen
 * @details
 * 1. Complete the core.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_GEOMETRY_CORE_HPP_
#define ROBOT_UTILS_GEOMETRY_CORE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <type_traits>

#include "robot_utils/core/math_tools.hpp"
#include "robot_utils/core/typedef.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Convert skew symmetric matrix to vector
 * @tparam T Type of the value, should be floating point
 * @param[in] v: Vector
 * @param[out] S: Skew symmetric matrix of v
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Vec2Skew(const Vector3<T>& v, Matrix3<T>& S)
{
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @tparam T Type of the value, should be floating point
 * @param[in] v: Vector
 * @return Skew symmetric matrix of v
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Matrix3<T> Vec2Skew(const Vector3<T>& v)
{
  Matrix3<T> S;
  Vec2Skew(v, S);
  return S;
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @tparam T Type of the value, should be floating point
 * @param[in] S: Skew symmetric matrix
 * @param[out] v: The vector corresponding to S
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Skew2Vec(const Matrix3<T>& S, Vector3<T>& v)
{
  v(0) = S(2, 1);
  v(1) = S(0, 2);
  v(2) = S(1, 0);
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @tparam T Type of the value, should be floating point
 * @param[in] S: Skew symmetric matrix
 * @return The vector corresponding to S
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Vector3<T> Skew2Vec(const Matrix3<T>& S)
{
  Vector3<T> v;
  Skew2Vec(S, v);
  return v;
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @param[out] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @note When the pitch angle is close to +/-90 degrees, the roll and yaw angles
 * are not unique. This function will return the roll angles as 0.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2EulerAngle(const Matrix3<T>& R,
                              Vector3<T>& euler)
{
  euler.y() = -std::asin(R(2, 0));
  if (std::abs(R(2, 0) < T(0.9999999))) {  // not gimbal lock
    T sign = GetSign(std::cos(euler.y()));
    euler.x() = std::atan2(R(2, 1) * sign, R(2, 2) * sign);
    euler.z() = std::atan2(R(1, 0) * sign, R(0, 0) * sign);
  } else {
    euler.x() = 0;
    if (R(2, 0) > 0) {
      euler.z() = std::atan2(-R(0, 1), -R(0, 2));
    } else {
      euler.z() = -std::atan2(R(0, 1), R(0, 2));
    }
  }
}

/**
 * @brief Convert rotation matrix to Euler angles
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @note When the pitch angle is close to +/-90 degrees, the roll and yaw angles
 * are not unique. This function will return the roll angles as 0.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Vector3<T> RotMat2EulerAngle(const Matrix3<T>& R)
{
  Vector3<T> euler;
  RotMat2EulerAngle(R, euler);
  return euler;
}

/**
 * @brief Convert rotation matrix to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @param[out] q: Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2Quat(const Matrix3<T>& R, Eigen::Quaternion<T>& q)
{
  q = Eigen::Quaternion<T>(R);
  q.normalize();
}

/**
 * @brief Convert rotation matrix to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @return Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Quaternion<T> RotMat2Quat(const Matrix3<T>& R)
{
  Eigen::Quaternion<T> q;
  RotMat2Quat(R, q);
  return q;
}

/**
 * @brief Convert rotation matrix to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @param[out] angle_axis: Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2AngleAxis(const Matrix3<T>& R,
                             Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(R);
}

/**
 * @brief Convert rotation matrix to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] R: Rotation matrix
 * @return Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::AngleAxis<T> RotMat2AngleAxis(const Matrix3<T>& R)
{
  Eigen::AngleAxis<T> angle_axis;
  RotMat2AngleAxis(R, angle_axis);
  return angle_axis;
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] R: Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2RotMat(const Vector3<T>& euler,
                              Matrix3<T>& R)
{
  R = Eigen::AngleAxis<T>(euler.z(), Vector3<T>::UnitZ()) *
      Eigen::AngleAxis<T>(euler.y(), Vector3<T>::UnitY()) *
      Eigen::AngleAxis<T>(euler.x(), Vector3<T>::UnitX());
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Matrix3<T> EulerAngle2RotMat(const Vector3<T>& euler)
{
  Matrix3<T> R;
  EulerAngle2RotMat(euler, R);
  return R;
}

/**
 * @brief Convert Euler angles to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] q: Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2Quat(const Vector3<T>& euler,
                            Eigen::Quaternion<T>& q)
{
  q = Eigen::AngleAxis<T>(euler.z(), Vector3<T>::UnitZ()) *
      Eigen::AngleAxis<T>(euler.y(), Vector3<T>::UnitY()) *
      Eigen::AngleAxis<T>(euler.x(), Vector3<T>::UnitX());
  q.normalize();
}

/**
 * @brief Convert Euler angles to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Quaternion<T> EulerAngle2Quat(const Vector3<T>& euler)
{
  Eigen::Quaternion<T> q;
  EulerAngle2Quat(euler, q);
  return q;
}

/**
 * @brief Convert Euler angles to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] angle_axis: Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2AngleAxis(const Vector3<T>& euler,
                                 Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(euler.z(), Vector3<T>::UnitZ()) *
               Eigen::AngleAxis<T>(euler.y(), Vector3<T>::UnitY()) *
               Eigen::AngleAxis<T>(euler.x(), Vector3<T>::UnitX());
}

/**
 * @brief Convert Euler angles to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::AngleAxis<T> EulerAngle2AngleAxis(const Vector3<T>& euler)
{
  Eigen::AngleAxis<T> angle_axis;
  EulerAngle2AngleAxis(euler, angle_axis);
  return angle_axis;
}

/**
 * @brief Convert quaternion to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @param[out] R: Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2RotMat(const Eigen::Quaternion<T>& q, Matrix3<T>& R)
{
  R = q.normalized().toRotationMatrix();
}

/**
 * @brief Convert quaternion to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @return Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Matrix3<T> Quat2RotMat(const Eigen::Quaternion<T>& q)
{
  Matrix3<T> R;
  Quat2RotMat(q, R);
  return R;
}

/**
 * @brief Convert quaternion to Euler angles
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @param[out] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2EulerAngle(const Eigen::Quaternion<T>& q,
                            Vector3<T>& euler)
{
  Eigen::Quaternion<T> qn = q.normalized();
  euler.x() = std::atan2(qn.w() * qn.x() + qn.y() * qn.z(),
                         T(0.5) - qn.x() * qn.x() - qn.y() * qn.y());

  T sinp = 2 * (qn.w() * qn.y() - qn.z() * qn.x());
  if (std::abs(sinp) >= 1) {  // gimbal lock
    euler.y() = std::copysign(M_PI_2, sinp);
  } else {
    euler.y() = std::asin(sinp);
  }

  euler.z() = std::atan2(qn.w() * qn.z() + qn.x() * qn.y(),
                         T(0.5) - qn.y() * qn.y() - qn.z() * qn.z());
}

/**
 * @brief Convert quaternion to Euler angles
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Vector3<T> Quat2EulerAngle(const Eigen::Quaternion<T>& q)
{
  Vector3<T> euler;
  Quat2EulerAngle(q, euler);
  return euler;
}

/**
 * @brief Convert quaternion to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @param[out] angle_axis: Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2AngleAxis(const Eigen::Quaternion<T>& q,
                           Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(q.normalized());
}

/**
 * @brief Convert quaternion to angle-axis representation
 * @tparam T Type of the value, should be floating point
 * @param[in] q: Quaternion
 * @return Angle-axis representation
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::AngleAxis<T> Quat2AngleAxis(const Eigen::Quaternion<T>& q)
{
  Eigen::AngleAxis<T> angle_axis;
  Quat2AngleAxis(q, angle_axis);
  return angle_axis;
}

/**
 * @brief Convert angle-axis representation to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @param[out] R: Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleAxis2RotMat(const Eigen::AngleAxis<T>& angle_axis,
                             Matrix3<T>& R)
{
  R = angle_axis.toRotationMatrix();
}

/**
 * @brief Convert angle-axis representation to rotation matrix
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @return Rotation matrix
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Matrix3<T> AngleAxis2RotMat(const Eigen::AngleAxis<T>& angle_axis)
{
  Matrix3<T> R;
  AngleAxis2RotMat(angle_axis, R);
  return R;
}

/**
 * @brief Convert angle-axis representation to Euler angles
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @param[out] euler: Euler angles(ZYX) in radians, [roll, pitch, yaw]
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleAxis2EulerAngle(const Eigen::AngleAxis<T>& angle_axis,
                                 Vector3<T>& euler)
{
  Matrix3<T> R = AngleAxis2RotMat(angle_axis);
  RotMat2EulerAngle(R, euler);
}

/**
 * @brief Convert angle-axis representation to Euler angles
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Vector3<T> AngleAxis2EulerAngle(
    const Eigen::AngleAxis<T>& angle_axis)
{
  Vector3<T> euler;
  AngleAxis2EulerAngle(angle_axis, euler);
  return euler;
}

/**
 * @brief Convert angle-axis representation to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @param[out] q: Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleAxis2Quat(const Eigen::AngleAxis<T>& angle_axis,
                           Eigen::Quaternion<T>& q)
{
  q = Eigen::Quaternion<T>(angle_axis);
  q.normalize();
}

/**
 * @brief Convert angle-axis representation to quaternion
 * @tparam T Type of the value, should be floating point
 * @param[in] angle_axis: Angle-axis representation
 * @return Quaternion
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Quaternion<T> AngleAxis2Quat(
    const Eigen::AngleAxis<T>& angle_axis)
{
  Eigen::Quaternion<T> q;
  AngleAxis2Quat(angle_axis, q);
  return q;
}
}  // namespace robot_utils

#endif /* ROBOT_UTILS_GEOMETRY_CORE_HPP_ */
