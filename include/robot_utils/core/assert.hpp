/**
 *******************************************************************************
 * @file      : assert.hpp
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
#ifndef ROBOT_UTILS_CORE_ASSERT_HPP_
#define ROBOT_UTILS_CORE_ASSERT_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdarg>
#include <cstdio>

/* Exported macro ------------------------------------------------------------*/
#if defined(USE_RU_ASSERT) && USE_RU_ASSERT
#define PARAM_ASSERT(expr, format, ...) \
  ((expr)                               \
       ? (void)0U                       \
       : robot_utils::AssertFailed(__FILE__, __LINE__, format, ##__VA_ARGS__))
#else
#define PARAM_ASSERT(expr, format, ...) ((void)0U)
#endif

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

static void AssertFailed(const char* file, int line, const char* format, ...)
{
  va_list args;
  va_start(args, format);

  fprintf(stderr, "\033[31mAssert failed at %s: %d: ", file, line);
  vfprintf(stderr, format, args);
  fprintf(stderr, "\033[0m\n");

  va_end(args);
}
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CORE_ASSERT_HPP_ */
