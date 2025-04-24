/**
 *******************************************************************************
 * @file      : td.hpp
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
#ifndef ROBOT_UTILS_FILTER_TD_HPP_
#define ROBOT_UTILS_FILTER_TD_HPP_

/* Includes ------------------------------------------------------------------*/
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <memory>
#include <type_traits>
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template <typename T>
struct TdParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "TdParams only supports float and double");

  T cutoff_freq = 0;  //<! cutoff frequency of tracking-differentiator
  T dt = 0;           //<! sampling time

  /*<! optimization parameters */

  T period = 0;  //<! period of data, <= 0 means no periodic data

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * cutoff_freq: <value>
   * dt: <value>
   * period: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: TdParams object to store the loaded parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(YAML::Node& node, TdParams& params)
  {
    params.cutoff_freq = node["cutoff_freq"].as<T>();
    params.dt = node["dt"].as<T>();
    params.period = node["period"].as<T>();
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * cutoff_freq: <value>
   * dt: <value>
   * period: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return TdParams object containing the loaded parameters
   * @note None
   */
  static TdParams LoadParamsFromYamlNode(YAML::Node& node)
  {
    TdParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Tracking Differentiator
 *
 * This class implements a tracking differentiator (TD) for estimating the
 * derivative of a signal. The TD is designed to provide a smooth
 * estimation of the derivative while being robust to noise and
 * disturbances.
 *
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class Td
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "Td only supports float and double");

 public:
  using Params = TdParams<T>;
  using Ptr = std::shared_ptr<Td<T>>;
  using ConstPtr = std::shared_ptr<const Td<T>>;

  struct Data {
    T x = 0;
    T dx = 0;

    bool is_first_calc = true;
  };

  explicit Td(const Params& params);
  ~Td(void) = default;

  /**
   * @brief Calculate the derivative of the input signal
   *
   * This function calculates the derivative of the input signal using the
   * tracking differentiator algorithm.
   *
   * @param[in] x: Input signal
   * @return The estimated derivative of the input signal
   * @note None
   */
  T calc(const T& x);

  void reset(void) { data_ = Data(); }

  bool isDiverged(void) const
  {
    return std::isnan(data_.x) || std::isnan(data_.dx) || std::isinf(data_.x) ||
           std::isinf(data_.dx);
  }

  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

  const Data& getData(void) const { return data_; }

 private:
  Params params_;
  Data data_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_FILTER_TD_HPP_ */
