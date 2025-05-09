/**
 *******************************************************************************
 * @file td.hpp
 * @brief Tracking Differentiator
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-08
 * @author Caikunzhen
 * @details
 * 1. Complete the td.hpp
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

/**
 * @brief Parameters of tracking differentiator
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
struct TdParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "TdParams only supports float and double");

  /**
   * @brief cutoff frequency, \f$r\f$, unit: Hz
   *
   * @note Must be smaller than \f$0.5/\Delta t\f$, otherwise it is easy to
   * cause divergence.
   */
  T cutoff_freq = 0;
  T dt = 0;  ///< sampling time, \f$\Delta t\f$, unit: s

  /* optimization parameters */

  /// period of data, \f$T\f$, \f$T\le 0\f$ means non periodic data
  T period = 0;

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
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node, TdParams& params)
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
   */
  static TdParams LoadParamsFromYamlNode(const YAML::Node& node)
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

  /**
   * @brief Data of tracking differentiator
   */
  struct Data {
    T x = 0;  ///< the filtering result of the input signal, \f$\hat x\f$
    /// the estimate derivative of the input signal, \f$\dot{\hat x}\f$
    T dx = 0;

    bool is_first_calc = true;  ///< whether the first calculation
  };

  explicit Td(const Params& params);
  virtual ~Td(void) = default;

  /**
   * @brief Calculate the derivative of the input signal
   *
   * This function calculates the derivative of the input signal using the
   * tracking differentiator algorithm.
   *
   * \f[
   * G\left(s\right) = \frac{r^2}{\left(s+r\right)^2}
   * \f]
   *
   * \f[
   * \hat X\left(s\right) = G\left(s\right) * X\left(s\right)
   * \f]
   *
   * \f[
   * \dot{\hat X}\left(s\right) = s * \hat X\left(s\right)
   * \f]
   *
   * @param[in] x: Input signal, \f$x\f$
   * @return The estimated derivative of the input signal, \f$\dot{\hat x}\f$
   * @note It is recommended to call the @ref isDiverged method before using the
   * result to determine whether it is diverged. If it is diverged, call the
   * @ref reset method to reset the TD.
   */
  T calc(const T& x);

  /**
   * @brief Reset the TD
   */
  void reset(void) { data_ = Data(); }

  /**
   * @brief Check if the TD is diverged
   * @return true if the TD is diverged, false otherwise
   * @note When the TD is diverged, the output may be NaN or Inf. In this case,
   * it is recommended to call the @ref reset method to reset the TD.
   */
  bool isDiverged(void) const
  {
    return std::isnan(data_.x) || std::isnan(data_.dx) || std::isinf(data_.x) ||
           std::isinf(data_.dx);
  }

  /**
   * @brief Set the parameters of the TD
   * @param params Parameters of the TD
   * @note params.dt and params.period will be ignored.
   */
  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

  const Data& getData(void) const { return data_; }

 private:
  Params params_;
  Data data_;
};

extern template class Td<float>;
using Tdf = Td<float>;
extern template class Td<double>;
using Tdd = Td<double>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_FILTER_TD_HPP_ */
