/**
 *******************************************************************************
 * @file      : pid.hpp
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
#ifndef ROBOT_UTILS_CONTROLLER_PID_HPP_
#define ROBOT_UTILS_CONTROLLER_PID_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <vector>

#include "robot_utils/controller/pid_node.hpp"
#include "robot_utils/core/assert.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

template <typename T>
struct CascadedPidBaseParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CascadedPidBaseParams only supports float and double");

  T ctrl_rate = 0;
  size_t n_pid = 0;  //<! number of pid nodes

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * n_pid: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: CascadedPidBaseParams object to store the loaded
   * parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(YAML::Node& node,
                                     CascadedPidBaseParams& params)
  {
    params.ctrl_rate = node["ctrl_rate"].as<T>();
    params.n_pid = node["n_pid"].as<size_t>();
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * n_pid: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return CascadedPidBaseParams object containing the loaded parameters
   * @note None
   */
  static CascadedPidBaseParams LoadParamsFromYamlNode(YAML::Node& node)
  {
    CascadedPidBaseParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

template <typename T>
struct CascadedPidParams : public CascadedPidBaseParams<T> {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CascadedPidParams only supports float and double");

  /**<! parameters of pid nodes, node_params[i].ctrl_rate = ctrl_rate */
  std::vector<PidNodeParams<T>> node_params;

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * n_pid: <value>
   *
   * node_params:
   *   - kp: <value>
   *     ki: <value>
   *     kd: <value>
   *     ctrl_rate: <value>  # not used
   *
   *     period: <value>
   *     out_limit_lb: <value>
   *     out_limit_ub: <value>
   *     deadband_lb: <value>
   *     deadband_ub: <value>
   *     en_trap_int: <value>
   *     anti_windup_lb: <value>
   *     anti_windup_up: <value>
   *     int_separate_lb: <value>
   *     int_separate_ub: <value>
   *     perv_diff_weight: <value>
   *     td_params:
   *       cutoff_freq: <value>
   *       dt: <value>      # not used
   *       period: <value>  # not used
   *
   *     en_out_limit: <value>
   *     en_deadband: <value>
   *     en_trap_int: <value>
   *     en_anti_windup: <value>
   *     en_int_separate: <value>
   *     en_td: <value>
   *   - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: CascadedPidParams object to store the loaded
   * parameters
   * @return None
   * @note The size of the node_params vector should be equal to n_pid.
   */
  static void LoadParamsFromYamlNode(YAML::Node& node,
                                     CascadedPidParams& params)
  {
    CascadedPidBaseParams<T>::LoadParamsFromYamlNode(node, params);
    params.pid_node_params.resize(params.n_pid);

    PARAM_ASSERT(params.n_pid == node["node_params"].size(),
                 "The size of node_params should be equal to n_pid");

    for (size_t i = 0; i < params.n_pid; ++i) {
      PidNodeParams<T>::LoadParamsFromYamlNode(node["node_params"][i],
                                               params.node_params[i]);
      params.node_params[i].ctrl_rate = params.ctrl_rate;
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * n_pid: <value>
   *
   * node_params:
   *   - kp: <value>
   *     ki: <value>
   *     kd: <value>
   *     ctrl_rate: <value>  # not used
   *
   *     period: <value>
   *     out_limit_lb: <value>
   *     out_limit_ub: <value>
   *     deadband_lb: <value>
   *     deadband_ub: <value>
   *     en_trap_int: <value>
   *     anti_windup_lb: <value>
   *     anti_windup_up: <value>
   *     int_separate_lb: <value>
   *     int_separate_ub: <value>
   *     perv_diff_weight: <value>
   *     td_params:
   *       cutoff_freq: <value>
   *       dt: <value>      # not used
   *       period: <value>  # not used
   *
   *     en_out_limit: <value>
   *     en_deadband: <value>
   *     en_trap_int: <value>
   *     en_anti_windup: <value>
   *     en_int_separate: <value>
   *     en_td: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return CascadedPidParams object containing the loaded parameters
   * @note The size of the node_params vector should be equal to n_pid.
   */
  static CascadedPidParams LoadParamsFromYamlNode(YAML::Node& node)
  {
    CascadedPidParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

template <typename T>
struct ParallelPidBaseParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ParallelPidBaseParams only supports float and double");

  T ctrl_rate = 0;
  T out_limit_lb = 0;
  T out_limit_ub = 0;
  size_t n_pid = 0;  //<! number of pid nodes

  bool en_out_limit = false;

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * n_pid: <value>
   * en_out_limit: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: ParallelPidParams object to store the loaded
   * parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(YAML::Node& node,
                                     ParallelPidBaseParams& params)
  {
    params.ctrl_rate = node["ctrl_rate"].as<T>();
    params.out_limit_lb = node["out_limit_lb"].as<T>();
    params.out_limit_ub = node["out_limit_ub"].as<T>();
    params.n_pid = node["n_pid"].as<size_t>();
    params.en_out_limit = node["en_out_limit"].as<bool>();
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * n_pid: <value>
   * en_out_limit: <value>
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return ParallelPidParams object containing the loaded parameters
   * @note None
   */
  static ParallelPidBaseParams LoadParamsFromYamlNode(YAML::Node& node)
  {
    ParallelPidBaseParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

template <typename T>
struct ParallelPidParams : public ParallelPidBaseParams<T> {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ParallelPidParams only supports float and double");

  /**<! parameters of pid nodes, node_params[i].ctrl_rate = ctrl_rate */
  std::vector<PidNodeParams<T>> node_params;

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * n_pid: <value>
   * en_out_limit: <value>
   *
   * node_params:
   *   - kp: <value>
   *     ki: <value>
   *     kd: <value>
   *     ctrl_rate: <value>  # not used
   *
   *     period: <value>
   *     out_limit_lb: <value>
   *     out_limit_ub: <value>
   *     deadband_lb: <value>
   *     deadband_ub: <value>
   *     en_trap_int: <value>
   *     anti_windup_lb: <value>
   *     anti_windup_up: <value>
   *     int_separate_lb: <value>
   *     int_separate_ub: <value>
   *     perv_diff_weight: <value>
   *     td_params:
   *       cutoff_freq: <value>
   *       dt: <value>      # not used
   *       period: <value>  # not used
   *
   *     en_out_limit: <value>
   *     en_deadband: <value>
   *     en_trap_int: <value>
   *     en_anti_windup: <value>
   *     en_int_separate: <value>
   *     en_td: <value>
   *  - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: ParallelPidNodeParams object to store the loaded
   * parameters
   * @return None
   * @note The size of the node_params vector should be equal to n_pid.
   */
  static void LoadParamsFromYamlNode(YAML::Node& node,
                                     ParallelPidParams& params)
  {
    ParallelPidBaseParams<T>::LoadParamsFromYamlNode(node, params);
    params.node_params.resize(params.n_pid);

    PARAM_ASSERT(params.n_pid == node["node_params"].size(),
                 "The size of node_params should be equal to n_pid");

    for (size_t i = 0; i < params.n_pid; ++i) {
      PidNodeParams<T>::LoadParamsFromYamlNode(node["node_params"][i],
                                               params.node_params[i]);
      params.node_params[i].ctrl_rate = params.ctrl_rate;
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * ctrl_rate: <value>
   * out_limit_lb: <value>
   * out_limit_ub: <value>
   * n_pid: <value>
   * en_out_limit: <value>
   *
   * node_params:
   *   - kp: <value>
   *     ki: <value>
   *     kd: <value>
   *     ctrl_rate: <value>  # not used
   *
   *     period: <value>
   *     out_limit_lb: <value>
   *     out_limit_ub: <value>
   *     deadband_lb: <value>
   *     deadband_ub: <value>
   *     en_trap_int: <value>
   *     anti_windup_lb: <value>
   *     anti_windup_up: <value>
   *     int_separate_lb: <value>
   *     int_separate_ub: <value>
   *     perv_diff_weight: <value>
   *     td_params:
   *       cutoff_freq: <value>
   *       dt: <value>      # not used
   *       period: <value>  # not used
   *
   *     en_out_limit: <value>
   *     en_deadband: <value>
   *     en_trap_int: <value>
   *     en_anti_windup: <value>
   *     en_int_separate: <value>
   *     en_td: <value>
   *  - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return ParallelPidNodeParams object containing the loaded parameters
   * @note The size of the node_params vector should be equal to n_pid.
   */
  static ParallelPidParams LoadParamsFromYamlNode(YAML::Node& node)
  {
    ParallelPidParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Cascaded PID controller class
 *
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class CascadedPid
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CascadedPid only supports float and double");

 public:
  using BaseParams = CascadedPidBaseParams<T>;
  using Params = CascadedPidParams<T>;
  using NodeParams = PidNodeParams<T>;
  using NodeParamsList = std::vector<NodeParams>;
  using Data = PidNode<T>::Data;
  using DataList = std::vector<Data>;

  using Ptr = std::shared_ptr<CascadedPid<T>>;
  using ConstPtr = std::shared_ptr<const CascadedPid<T>>;

  /*<! feedback vector, size = n_pid */
  using FdbVec = Eigen::VectorX<T>;
  //<! feedback vector, size = n_pid
  using FfdVec = Eigen::VectorX<T>;

  explicit CascadedPid(const Params& params);
  virtual ~CascadedPid(void) = default;

  /**
   * @brief Calculate the output of the cascaded PID controller
   * @param[in] ref: reference value
   * @param[in] fdb_vec: feedback vector, size = n_pid
   * @return output value
   * @note None
   */
  T calc(const T& ref, const FdbVec& fdb_vec);

  /**
   * @brief Calculate the output of the cascaded PID controller with feedforward
   * @param[in] ref: reference value
   * @param[in] fdb_vec: feedback vector, size = n_pid
   * @param[in] ffd_vec: feedforward vector, size = n_pid
   * @return output value
   * @note None
   */
  T calc(const T& ref, const FdbVec& fdb_vec, const FfdVec& ffd_vec);

  void reset(void)
  {
    for (auto& pid_node : pid_nodes_) {
      pid_node.reset();
    }
  }

  const BaseParams& getBaseParams(void) const { return base_params_; }

  /**
   * @brief Set the parameters of the PID nodes
   * @param[in] node_params_list: list of PID node parameters, size = n_pid
   * @return None
   * @note None
   */
  void setNodeParams(const NodeParamsList& node_params_list);

  /**
   * @brief Set the parameters of a specific PID node
   * @param[in] i: index of the PID node, 0 <= i < n_pid
   * @param[in] node_params: parameters of the PID node
   * @return None
   * @note None
   */
  void setNodeParamsAt(size_t i, const NodeParams& node_params);

  /**
   * @brief Get a specific PID node
   * @param[in] i: index of the PID node, 0 <= i < n_pid
   * @return PID node
   * @note None
   */
  const PidNode<T>& getNode(size_t i) const { return pid_nodes_[i]; }

 private:
  BaseParams base_params_;

  std::vector<PidNode<T>> pid_nodes_;  //<! pid nodes
};

/**
 * @brief Parallel PID controller class
 *
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class ParallelPid
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ParallelPid only supports float and double");

 public:
  using BaseParams = ParallelPidBaseParams<T>;
  using Params = ParallelPidParams<T>;
  using NodeParams = PidNodeParams<T>;
  using NodeParamsList = std::vector<NodeParams>;
  using Data = PidNode<T>::Data;
  using DataList = std::vector<Data>;

  /*<! reference vector, size = n_pid */
  using RefVec = Eigen::VectorX<T>;
  /*<! feedback vector, size = n_pid */
  using FdbVec = Eigen::VectorX<T>;
  //<! feedback vector, size = n_pid
  using FfdVec = Eigen::VectorX<T>;

  explicit ParallelPid(const Params& params);
  virtual ~ParallelPid(void) = default;

  /**
   * @brief Calculate the output of the parallel PID controller
   * @param[in] ref_vec: reference vector, size = n_pid
   * @param[in] fdb_vec: feedback vector, size = n_pid
   * @return output value
   * @note None
   */
  T calc(const RefVec& ref_vec, const FdbVec& fdb_vec);

  /**
   * @brief Calculate the output of the parallel PID controller with feedforward
   * @param[in] ref_vec: reference vector, size = n_pid
   * @param[in] fdb_vec: feedback vector, size = n_pid
   * @param[in] ffd_vec: feedforward vector, size = n_pid
   * @return output value
   * @note None
   */
  T calc(const RefVec& ref_vec, const FdbVec& fdb_vec, const FfdVec& ffd_vec);

  void reset(void)
  {
    for (auto& pid_node : pid_nodes_) {
      pid_node.reset();
    }
  }

  const BaseParams& getBaseParams(void) const { return base_params_; }

  /**
   * @brief Set the parameters of the PID nodes
   * @param[in] node_params_list: list of PID node parameters, size = n_pid
   * @return None
   * @note None
   */
  void setNodeParams(const NodeParamsList& node_params_list);

  /**
   * @brief Set the parameters of a specific PID node
   * @param[in] i: index of the PID node, 0 <= i < n_pid
   * @param[in] node_params: parameters of the PID node
   * @return None
   * @note None
   */
  void setNodeParamsAt(size_t i, const NodeParams& node_params);

  /**
   * @brief Get a specific PID node
   * @param[in] i: index of the PID node, 0 <= i < n_pid
   * @return PID node
   * @note None
   */
  const PidNode<T>& getNode(size_t i) const { return pid_nodes_[i]; }

 private:
  BaseParams base_params_;

  std::vector<PidNode<T>> pid_nodes_;  //<! pid nodes
};

extern template class CascadedPid<float>;
using CascadedPidf = CascadedPid<float>;
extern template class CascadedPid<double>;
using CascadedPidd = CascadedPid<double>;
extern template class ParallelPid<float>;
using ParallelPidf = ParallelPid<float>;
extern template class ParallelPid<double>;
using ParallelPidd = ParallelPid<double>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CONTROLLER_PID_HPP_ */
