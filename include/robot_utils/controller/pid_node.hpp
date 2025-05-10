/**
 *******************************************************************************
 * @file pid_node.hpp
 * @brief Basic PID controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-08
 * @author Caikunzhen
 * @details
 * 1. Complete the pid_node.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CONTROLLER_PID_NODE_HPP_
#define ROBOT_UTILS_CONTROLLER_PID_NODE_HPP_

/* Includes ------------------------------------------------------------------*/
#include <yaml-cpp/yaml.h>

#include <memory>
#include <type_traits>

#include "robot_utils/filter/td.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Parameters of basic PID controller
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
struct PidNodeParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "PidNodeParams only supports float and double");

  T kp = 0;  ///< proportional gain, \f$K_p\f$
  T ki = 0;  ///< integral gain, \f$K_i\f$
  T kd = 0;  ///< derivative gain, \f$K_d\f$
  T dt = 0;  ///< sampling time, \f$\Delta t\f$, unit: s

  /* optimization parameters */

  /// period of data, \f$T\f$, \f$T\le 0\f$ means non periodic data
  T period = 0;
  T out_limit_lb = 0;    ///< lower bound of output, \f$u_{min}\f$
  T out_limit_ub = 0;    ///< upper bound of output, \f$u_{max}\f$
  T deadband_lb = 0;     ///< lower bound of deadband, \f$\epsilon_{min}\f$
  T deadband_ub = 0;     ///< upper bound of deadband, \f$\epsilon_{max}\f$
  T anti_windup_lb = 0;  ///< lower bound of anti-windup, \f$u_{min}^{i}\f$
  T anti_windup_ub = 0;  ///< upper bound of anti-windup, \f$u_{max}^{i}\f$
  /// lower bound of integral separate, \f$\epsilon_{min}^{i}\f$
  T int_separate_lb = 0;
  /// upper bound of integral separate, \f$\epsilon_{max}^{i}\f$
  T int_separate_ub = 0;
  /**
   * @brief weight of pervious difference, \f$w_{prev}^{d}\f$, 0 means no
   * previous difference, 1 means only previous difference
   */
  T perv_diff_weight = 0;
  /// cutoff frequency of tracking-differentiator, \f$r\f$, unit: Hz
  T td_cutoff_freq = 0;

  /* switch of optimization */

  bool en_out_limit = false;     ///< enable output limit
  bool en_deadband = false;      ///< enable deadband
  bool en_trap_int = false;      ///< enable trapezoidal integral
  bool en_anti_windup = false;   ///< enable anti-windup
  bool en_int_separate = false;  ///< enable integral separate
  bool en_td = false;            ///< enable tracking-differentiator

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * kp: 0
   * ki: 0
   * kd: 0
   * dt: 0
   * period: 0
   *
   * en_out_limit: false
   * # optional, can be deleted if en_out_limit is false
   * out_limit_lb: 0
   * out_limit_ub: 0
   *
   * en_deadband: false
   * # optional, can be deleted if en_deadband is false
   * deadband_lb: 0
   * deadband_ub: 0
   *
   * en_trap_int: false
   *
   * en_anti_windup: false
   * # optional, need to be set if en_anti_windup is true
   * anti_windup_lb: 0
   * anti_windup_ub: 0
   *
   * en_int_separate: false
   * # optional, need to be set if en_int_separate is true
   * int_separate_lb: 0
   * int_separate_ub: 0
   *
   * perv_diff_weight: 0
   *
   * en_td: false
   * # optional, need to be set if en_td is true
   * td_cutoff_freq: 0
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: PidNodeParams object to store the loaded parameters
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node,
                                     PidNodeParams& params)
  {
    params.kp = node["kp"].as<T>();
    params.ki = node["ki"].as<T>();
    params.kd = node["kd"].as<T>();
    params.dt = node["dt"].as<T>();
    params.period = node["period"].as<T>();

    params.en_out_limit = node["en_out_limit"].as<bool>();
    if (params.en_out_limit) {
      params.out_limit_lb = node["out_limit_lb"].as<T>();
      params.out_limit_ub = node["out_limit_ub"].as<T>();
    }
    params.en_deadband = node["en_deadband"].as<bool>();
    if (params.en_deadband) {
      params.deadband_lb = node["deadband_lb"].as<T>();
      params.deadband_ub = node["deadband_ub"].as<T>();
    }
    params.en_trap_int = node["en_trap_int"].as<bool>();
    params.en_anti_windup = node["en_anti_windup"].as<bool>();
    if (params.en_anti_windup) {
      params.anti_windup_lb = node["anti_windup_lb"].as<T>();
      params.anti_windup_ub = node["anti_windup_ub"].as<T>();
    }
    params.en_int_separate = node["en_int_separate"].as<bool>();
    if (params.en_int_separate) {
      params.int_separate_lb = node["int_separate_lb"].as<T>();
      params.int_separate_ub = node["int_separate_ub"].as<T>();
    }
    params.perv_diff_weight = node["perv_diff_weight"].as<T>();
    params.en_td = node["en_td"].as<bool>();
    if (params.en_td) {
      params.td_cutoff_freq = node["td_cutoff_freq"].as<T>();
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * kp: 0
   * ki: 0
   * kd: 0
   * dt: 0
   * period: 0
   *
   * en_out_limit: false
   * # optional, can be deleted if en_out_limit is false
   * out_limit_lb: 0
   * out_limit_ub: 0
   *
   * en_deadband: false
   * # optional, can be deleted if en_deadband is false
   * deadband_lb: 0
   * deadband_ub: 0
   *
   * en_trap_int: false
   *
   * en_anti_windup: false
   * # optional, need to be set if en_anti_windup is true
   * anti_windup_lb: 0
   * anti_windup_ub: 0
   *
   * en_int_separate: false
   * # optional, need to be set if en_int_separate is true
   * int_separate_lb: 0
   * int_separate_ub: 0
   *
   * perv_diff_weight: 0
   *
   * en_td: false
   * # optional, need to be set if en_td is true
   * td_cutoff_freq: 0
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   */
  static PidNodeParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    PidNodeParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Basic PID controller class
 *
 * This class implements a basic PID controller, it is used to combine with each
 * other to form a complex PID controller.
 *
 * @tparam T Type of the data (only provides float and double)
 * @note It is not recommended to use this class alone.
 */
template <typename T>
class PidNode
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "PidNode only supports float and double");

 public:
  using Params = PidNodeParams<T>;
  using Ptr = std::shared_ptr<PidNode<T>>;
  using ConstPtr = std::shared_ptr<const PidNode<T>>;

  /**
   * @brief Data of basic PID controller
   */
  struct Data {
    T ref = 0;  ///< reference value, \f$x_i^d\f$
    T fdb = 0;  ///< feedback value, \f$x_i\f$
    T err = 0;  ///< error value, \f$e_i\f$
    T out = 0;  ///< output value, \f$u_i\f$

    T prev_ref = 0;  ///< previous reference value, \f$x_{i-1}^d\f$
    T prev_fdb = 0;  ///< previous feedback value, \f$x_{i-1}\f$
    T prev_err = 0;  ///< previous error value, \f$e_{i-1}\f$
    T prev_out = 0;  ///< previous output value, \f$u_{i-1}\f$

    T pout = 0;  ///< proportional output, \f$u_i^p\f$
    T iout = 0;  ///< integral output, \f$u_i^i\f$
    T dout = 0;  ///< derivative output, \f$u_i^d\f$

    bool is_first_calc = true;  ///< whether the first calculation
  };

  explicit PidNode(const Params& params);
  virtual ~PidNode(void) = default;

  /**
   * @brief Calculate the PID output
   *
   * Calculate the error
   * \f[
   * \hat e_i =
   * \begin{cases}
   * x_i^d - x_i, & T \leq 0 \\
   * {\rm PeriodicDataSub}\left(T, x_i^d, x_i\right), & \text{else}
   * \end{cases}
   * \f]
   *
   * \f[
   * e_i =
   * \begin{cases}
   * 0, & {\rm en\_deadband} = true \text{ and } \hat e_i \in
   * \left[\epsilon_{min}, \epsilon_{max}\right] \\
   * \hat e_i, &
   * \text{else}
   * \end{cases}
   * \f]
   *
   * Calculate the proportional term
   * \f[
   * u_i^p = K_p \cdot e_i
   * \f]
   *
   * Calculate the integral term
   * \f[
   * \hat e_i^i =
   * \begin{cases}
   * e_i, & {\rm en\_trap\_int} = false \\
   * \left(e_i + e_{i-1}\right)/2, & \text{else}
   * \end{cases}
   * \f]
   *
   * \f[
   * e_i^i =
   * \begin{cases}
   * 0, & {\rm en\_int\_separate} = true \text{ and } e_i^i \notin
   * [\epsilon_{min}^i, \epsilon_{max}^i] \\
   * \hat e_i^i, & \text{else}
   * \end{cases}
   * \f]
   *
   * \f[
   * u_i^i =
   * \begin{cases}
   * {\rm Clamp}\left(u_{i-1}^i + K_i \cdot \hat e_i^i \cdot \Delta t,
   * u_{min}^i, u_{max}^i\right), & {\rm en\_anti\_windup} = true \\
   * u_{i-1}^i + K_i \cdot \hat e_i^i \cdot \Delta t, & \text{else}
   * \end{cases}
   * \f]
   *
   * Calculate the derivative term
   * \f[
   * \hat e_i' =
   * \begin{cases}
   * \left(e_i - e_{i-1}\right)/\Delta t, & {\rm en\_td} = false \\
   * {\rm Td}\left(r, dt, 0, e_i\right), & \text{else}
   * \end{cases}
   * \f]
   *
   * \f[
   * x_i' =
   * \begin{cases}
   * \left(x_i - x_{i-1}\right)/\Delta t, & {\rm en\_td} = false \\
   * {\rm Td}\left(r, dt, T, x_i\right), & \text{else}
   * \end{cases}
   * \f]
   *
   * \f[
   * u_i^d = K_d \cdot \left(-w_{prev}^d \cdot x_i' + \left(1 -
   * w_{prev}^d\right) * \cdot \hat e_i'\right)
   * \f]
   *
   * Calculate the output
   * \f[
   * u_i =
   * \begin{cases}
   * {\rm Clamp}\left(u_i^p + u_i^i + u_i^d + u_i^{ff}, u_{min}, u_{max}\right),
   * & {\rm en\_out\_limit} = true \\
   * u_i^p + u_i^i + u_i^d + u_i^{ff}, & \text{else}
   * \end{cases}
   * \f]
   *
   * \f$ {\rm PeriodicDataSub}\f$ is a function that calculates the periodic
   * subtraction of the data, defined in @ref PeriodicDataSub.
   *
   * \f$ {\rm Clamp}\f$ is a function that clamps the data to the given range,
   * same as std::clamp.
   *
   * \f${\rm Td}\f$ is a function that calculates the output of the tracking
   * differentiator, detailed see @ref Td.
   *
   * @param[in] ref: Reference value, \f$x_i^d\f$
   * @param[in] fdb: Feedback value, \f$x_i\f$
   * @param[in] ffd: Feedforward value, \f$u_i^{ff}\f$
   * @return PID output, \f$u_i\f$
   */
  T calc(const T& ref, const T& fdb, const T& ffd = 0);

  void reset(void)
  {
    data_ = Data();

    if (err_td_ptr_) {
      err_td_ptr_->reset();
      fdb_td_ptr_->reset();
    }
  }

  /**
   * @brief Set the parameters of the PID controller
   * @param params: Parameters of the PID controller(`dt` and `period` will be
   * ignored)
   */
  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

  const Data& getData(void) const { return data_; }

 private:
  Params params_;
  Data data_;

  Td<T>::Ptr err_td_ptr_ = nullptr;
  Td<T>::Ptr fdb_td_ptr_ = nullptr;
};

extern template class PidNode<float>;
using PidNodef = PidNode<float>;
extern template class PidNode<double>;
using PidNoded = PidNode<double>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CONTROLLER_PID_NODE_HPP_ */
