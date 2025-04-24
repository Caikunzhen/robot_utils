/**
 *******************************************************************************
 * @file      : pid_node.hpp
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

template <typename T>
struct PidNodeParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "PidNodeParams only supports float and double");

  T kp = 0;
  T ki = 0;
  T kd = 0;
  T ctrl_rate = 0;

  /*<! optimization parameters */

  /*<! period of data, <= 0 means no periodic data */
  T period = 0;
  T out_limit_lb = 0;     //<! lower bound of output
  T out_limit_ub = 0;     //<! upper bound of output
  T deadband_lb = 0;      //<! lower bound of deadband
  T deadband_ub = 0;      //<! upper bound of deadband
  T anti_windup_lb = 0;   //<! lower bound of anti-windup
  T anti_windup_up = 0;   //<! upper bound of anti-windup
  T int_separate_lb = 0;  //<! lower bound of integral separate
  T int_separate_ub = 0;  //<! upper bound of integral separate
  /*<! weight of pervious difference, 0 means no previous difference, 1 means
   * only previous difference */
  T perv_diff_weight = 0;
  /*<! parameters of tracking-differentiator, td_params.period = period,
   * td_params.dt = 1 / ctrl_rate */
  TdParams<T> td_params;

  /*<! switch of optimization */

  bool en_out_limit = false;     //<! enable output limit
  bool en_deadband = false;      //<! enable deadband
  bool en_trap_int = false;      //<! enable trapezoidal integral
  bool en_anti_windup = false;   //<! enable anti-windup
  bool en_int_separate = false;  //<! enable integral separate
  bool en_td = false;            //<! enable tracking-differentiator

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * kp: <value>
   * ki: <value>
   * kd: <value>
   * ctrl_rate: <value>
   *
   * period: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * deadband_lb: <value>
   * deadband_ub: <value>
   * en_trap_int: <value>
   * anti_windup_lb: <value>
   * anti_windup_up: <value>
   * int_separate_lb: <value>
   * int_separate_ub: <value>
   * perv_diff_weight: <value>
   * td_params:
   *  cutoff_freq: <value>
   *  dt: <value>      # not used
   *  period: <value>  # not used
   *
   * en_out_limit: <value>
   * en_deadband: <value>
   * en_trap_int: <value>
   * en_anti_windup: <value>
   * en_int_separate: <value>
   * en_td: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: PidNodeParams object to store the loaded parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(YAML::Node& node, PidNodeParams& params)
  {
    params.kp = node["kp"].as<T>();
    params.ki = node["ki"].as<T>();
    params.kd = node["kd"].as<T>();
    params.ctrl_rate = node["ctrl_rate"].as<T>();

    params.period = node["period"].as<T>();
    params.out_limit_lb = node["out_limit_lb"].as<T>();
    params.out_limit_ub = node["out_limit_ub"].as<T>();
    params.deadband_lb = node["deadband_lb"].as<T>();
    params.deadband_ub = node["deadband_ub"].as<T>();
    params.anti_windup_lb = node["anti_windup_lb"].as<T>();
    params.anti_windup_up = node["anti_windup_up"].as<T>();
    params.int_separate_lb = node["int_separate_lb"].as<T>();
    params.int_separate_ub = node["int_separate_ub"].as<T>();
    params.perv_diff_weight = node["perv_diff_weight"].as<T>();
    TdParams<T>::LoadParamsFromYamlNode(node["td_params"], params.td_params);
    params.td_params.period = params.period;
    params.td_params.dt = 1 / params.ctrl_rate;

    params.en_out_limit = node["en_out_limit"].as<bool>();
    params.en_deadband = node["en_deadband"].as<bool>();
    params.en_trap_int = node["en_trap_int"].as<bool>();
    params.en_anti_windup = node["en_anti_windup"].as<bool>();
    params.en_int_separate = node["en_int_separate"].as<bool>();
    params.en_td = node["en_td"].as<bool>();
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * kp: <value>
   * ki: <value>
   * kd: <value>
   * ctrl_rate: <value>
   *
   * period: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * deadband_lb: <value>
   * deadband_ub: <value>
   * en_trap_int: <value>
   * anti_windup_lb: <value>
   * anti_windup_up: <value>
   * int_separate_lb: <value>
   * int_separate_ub: <value>
   * perv_diff_weight: <value>
   * td_params:
   *   cutoff_freq: <value>
   *   dt: <value>     # not used
   *   period: <value> # not used
   *
   * en_out_limit: <value>
   * en_deadband: <value>
   * en_trap_int: <value>
   * en_anti_windup: <value>
   * en_int_separate: <value>
   * en_td: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return None
   * @note None
   */
  static PidNodeParams LoadParamsFromYamlNode(YAML::Node& node)
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
 * other to form a complex PID controller. So it is not recommended to use it
 * alone.
 *
 * This class implements a PID controller with the following features:
 *
 * @tparam T Type of the data (only provides float and double)
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

  struct Data {
    T ref = 0;
    T fdb = 0;
    T err = 0;
    T out = 0;

    T prev_ref = 0;
    T prev_fdb = 0;
    T prev_err = 0;
    T prev_out = 0;

    T pout = 0;
    T iout = 0;
    T dout = 0;

    bool is_first_calc = true;
  };

  explicit PidNode(const Params& params);
  ~PidNode(void) = default;

  /**
   * @brief Calculate the PID output
   * @param[in] ref: reference value
   * @param[in] fdb: feedback value
   * @param[in] ffd: feedforward value
   * @return PID output
   * @note None
   */
  T calc(const T& ref, const T& fdb, const T& ffd = 0);

  void reset(void)
  {
    data_ = Data();

    if (td_ptr_) {
      td_ptr_->reset();
    }
  }

  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

  const Data& getData(void) const { return data_; }

  Td<T>::ConstPtr getTdPtr(void) const { return td_ptr_; }

 private:
  Params params_;
  Data data_;

  Td<T>::Ptr td_ptr_;
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CONTROLLER_PID_NODE_HPP_ */
