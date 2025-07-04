/**
 *******************************************************************************
 * @file typedef.hpp
 * @brief This file provides the typedefs for robot_utils.
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-07-04
 * @author Caikunzhen
 * @details
 * 1. Complete the typedef.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CORE_TYPEDEF_HPP_
#define ROBOT_UTILS_CORE_TYPEDEF_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CORE_TYPEDEF_HPP_ */
