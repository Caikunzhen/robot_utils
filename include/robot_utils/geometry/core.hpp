/**
 *******************************************************************************
 * @file      : core.hpp
 * @brief     : This file provides the basic function for geometry process.
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2025-04-30      Caikunzhen      1. Complete the core.hpp
 *******************************************************************************
 * @attention :
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
 * @brief Convert skew symmetric matrix to vector
 * @param[in] v Vector
 * @param[out] S Skew symmetric matrix
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Vec2Skew(const Eigen::Vector3<T>& v, Eigen::Matrix3<T>& S)
{
  S << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @param[in] v Vector
 * @return Skew symmetric matrix
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Matrix3<T> Vec2Skew(const Eigen::Vector3<T>& v)
{
  Eigen::Matrix3<T> S;
  Vec2Skew(v, S);
  return S;
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @param[in] S Skew symmetric matrix
 * @param[out] v Vector
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Skew2Vec(const Eigen::Matrix3<T>& S, Eigen::Vector3<T>& v)
{
  v(0) = S(2, 1);
  v(1) = S(0, 2);
  v(2) = S(1, 0);
}

/**
 * @brief Convert skew symmetric matrix to vector
 * @param[in] S Skew symmetric matrix
 * @return Vector
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Vector3<T> Skew2Vec(const Eigen::Matrix3<T>& S)
{
  Eigen::Vector3<T> v;
  Skew2Vec(S, v);
  return v;
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @param[in] R Rotation matrix
 * @param[out] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return None
 * @note When the pitch angle is close to +/-90 degrees, the roll and yaw angles
 * are not unique. This function will return the roll and yaw angles as 0.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2EulerAngle(const Eigen::Matrix3<T>& R,
                              Eigen::Vector3<T>& euler)
{
  euler.y() = -asin(R(2, 0));
  if (abs(R(2, 0) < T(0.9999999))) {  // not gimbal lock
    T sign = GetSign(cos(euler.y()));
    euler.x() = atan2(R(2, 1) * sign, R(2, 2) * sign);
    euler.z() = atan2(R(1, 0) * sign, R(0, 0) * sign);
  } else {
    euler.x() = 0;
    if (R(2, 0) > 0) {
      euler.z() = atan2(-R(0, 1), -R(0, 2));
    } else {
      euler.z() = -atan2(R(0, 1), R(0, 2));
    }
  }
}

/**
 * @brief Convert rotation matrix to Euler angles
 * @param[in] R Rotation matrix
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @note When the pitch angle is close to +/-90 degrees, the roll and yaw angles
 * are not unique. This function will return the roll and yaw angles as 0.
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Vector3<T> RotMat2EulerAngle(const Eigen::Matrix3<T>& R)
{
  Eigen::Vector3<T> euler;
  RotMat2EulerAngle(R, euler);
  return euler;
}

/**
 * @brief Convert rotation matrix to quaternion
 * @param[in] R Rotation matrix
 * @param[out] q Quaternion
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2Quat(const Eigen::Matrix3<T>& R, Eigen::Quaternion<T>& q)
{
  q = Eigen::Quaternion<T>(R);
  q.normalize();
}

/**
 * @brief Convert rotation matrix to quaternion
 * @param[in] R Rotation matrix
 * @return Quaternion
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Quaternion<T> RotMat2Quat(const Eigen::Matrix3<T>& R)
{
  Eigen::Quaternion<T> q;
  RotMat2Quat(R, q);
  return q;
}

/**
 * @brief Convert rotation matrix to angle-axis representation
 * @param[in] R Rotation matrix
 * @param[out] angle_axis Angle-axis representation
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void RotMat2AngleAxis(const Eigen::Matrix3<T>& R,
                             Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(R);
}

/**
 * @brief Convert rotation matrix to angle-axis representation
 * @param[in] R Rotation matrix
 * @return Angle-axis representation
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::AngleAxis<T> RotMat2AngleAxis(const Eigen::Matrix3<T>& R)
{
  Eigen::AngleAxis<T> angle_axis;
  RotMat2AngleAxis(R, angle_axis);
  return angle_axis;
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] R Rotation matrix
 * @return None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2RotMat(const Eigen::Vector3<T>& euler,
                              Eigen::Matrix3<T>& R)
{
  R = Eigen::AngleAxis<T>(euler.z(), Eigen::Vector3<T>::UnitZ()) *
      Eigen::AngleAxis<T>(euler.y(), Eigen::Vector3<T>::UnitY()) *
      Eigen::AngleAxis<T>(euler.x(), Eigen::Vector3<T>::UnitX());
}

/**
 * @brief Convert Euler angles to rotation matrix
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Rotation matrix
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Matrix3<T> EulerAngle2RotMat(const Eigen::Vector3<T>& euler)
{
  Eigen::Matrix3<T> R;
  EulerAngle2RotMat(euler, R);
  return R;
}

/**
 * @brief Convert Euler angles to quaternion
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] q Quaternion
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2Quat(const Eigen::Vector3<T>& euler,
                            Eigen::Quaternion<T>& q)
{
  q = Eigen::AngleAxis<T>(euler.z(), Eigen::Vector3<T>::UnitZ()) *
      Eigen::AngleAxis<T>(euler.y(), Eigen::Vector3<T>::UnitY()) *
      Eigen::AngleAxis<T>(euler.x(), Eigen::Vector3<T>::UnitX());
  q.normalize();
}

/**
 * @brief Convert Euler angles to quaternion
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Quaternion
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Quaternion<T> EulerAngle2Quat(const Eigen::Vector3<T>& euler)
{
  Eigen::Quaternion<T> q;
  EulerAngle2Quat(euler, q);
  return q;
}

/**
 * @brief Convert Euler angles to angle-axis representation
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @param[out] angle_axis Angle-axis representation
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void EulerAngle2AngleAxis(const Eigen::Vector3<T>& euler,
                                 Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(euler.z(), Eigen::Vector3<T>::UnitZ()) *
               Eigen::AngleAxis<T>(euler.y(), Eigen::Vector3<T>::UnitY()) *
               Eigen::AngleAxis<T>(euler.x(), Eigen::Vector3<T>::UnitX());
}

/**
 * @brief Convert Euler angles to angle-axis representation
 * @param[in] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return Angle-axis representation
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::AngleAxis<T> EulerAngle2AngleAxis(const Eigen::Vector3<T>& euler)
{
  Eigen::AngleAxis<T> angle_axis;
  EulerAngle2AngleAxis(euler, angle_axis);
  return angle_axis;
}

/**
 * @brief Convert quaternion to rotation matrix
 * @param[in] q Quaternion
 * @param[out] R Rotation matrix
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2RotMat(const Eigen::Quaternion<T>& q, Eigen::Matrix3<T>& R)
{
  R = q.normalized().toRotationMatrix();
}

/**
 * @brief Convert quaternion to rotation matrix
 * @param[in] q Quaternion
 * @return Rotation matrix
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Matrix3<T> Quat2RotMat(const Eigen::Quaternion<T>& q)
{
  Eigen::Matrix3<T> R;
  Quat2RotMat(q, R);
  return R;
}

/**
 * @brief Convert quaternion to Euler angles
 * @param[in] q Quaternion
 * @param[out] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2EulerAngle(const Eigen::Quaternion<T>& q,
                            Eigen::Vector3<T>& euler)
{
  Eigen::Quaternion<T> qn = q.normalized();
  euler.x() = atan2(qn.w() * qn.x() + qn.y() * qn.z(),
                    T(0.5) - qn.x() * qn.x() - qn.y() * qn.y());

  T sinp = 2 * (qn.w() * qn.y() - qn.z() * qn.x());
  if (abs(sinp) >= 1) {  // gimbal lock
    euler.y() = copysign(M_PI_2, sinp);
  } else {
    euler.y() = asin(sinp);
  }

  euler.z() = atan2(qn.w() * qn.z() + qn.x() * qn.y(),
                    T(0.5) - qn.y() * qn.y() - qn.z() * qn.z());
}

/**
 * @brief Convert quaternion to Euler angles
 * @param[in] q Quaternion
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Vector3<T> Quat2EulerAngle(const Eigen::Quaternion<T>& q)
{
  Eigen::Vector3<T> euler;
  Quat2EulerAngle(q, euler);
  return euler;
}

/**
 * @brief Convert quaternion to angle-axis representation
 * @param[in] q Quaternion
 * @param[out] angle_axis Angle-axis representation
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void Quat2AngleAxis(const Eigen::Quaternion<T>& q,
                           Eigen::AngleAxis<T>& angle_axis)
{
  angle_axis = Eigen::AngleAxis<T>(q.normalized());
}

/**
 * @brief Convert quaternion to angle-axis representation
 * @param[in] q Quaternion
 * @return Angle-axis representation
 * @note None
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
 * @param[in] angle_axis Angle-axis representation
 * @param[out] R Rotation matrix
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleAxis2RotMat(const Eigen::AngleAxis<T>& angle_axis,
                             Eigen::Matrix3<T>& R)
{
  R = angle_axis.toRotationMatrix();
}

/**
 * @brief Convert angle-axis representation to rotation matrix
 * @param[in] angle_axis Angle-axis representation
 * @return Rotation matrix
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Matrix3<T> AngleAxis2RotMat(const Eigen::AngleAxis<T>& angle_axis)
{
  Eigen::Matrix3<T> R;
  AngleAxis2RotMat(angle_axis, R);
  return R;
}

/**
 * @brief Convert angle-axis representation to Euler angles
 * @param[in] angle_axis Angle-axis representation
 * @param[out] euler Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @return None
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline void AngleAxis2EulerAngle(const Eigen::AngleAxis<T>& angle_axis,
                                 Eigen::Vector3<T>& euler)
{
  Eigen::Matrix3<T> R = AngleAxis2RotMat(angle_axis);
  RotMat2EulerAngle(R, euler);
}

/**
 * @brief Convert angle-axis representation to Euler angles
 * @param[in] angle_axis Angle-axis representation
 * @return Euler angles(ZYX) in radians, [roll, pitch, yaw]
 * @note None
 */
template <typename T, typename = std::enable_if_t<std::is_floating_point_v<T>>>
inline Eigen::Vector3<T> AngleAxis2EulerAngle(
    const Eigen::AngleAxis<T>& angle_axis)
{
  Eigen::Vector3<T> euler;
  AngleAxis2EulerAngle(angle_axis, euler);
  return euler;
}

/**
 * @brief Convert angle-axis representation to quaternion
 * @param[in] angle_axis Angle-axis representation
 * @param[out] q Quaternion
 * @return None
 * @note None
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
 * @param[in] angle_axis Angle-axis representation
 * @return Quaternion
 * @note None
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
