/**
 *******************************************************************************
 * @file pid.hpp
 * @brief Cascaded PID and parallel PID controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-09
 * @author Caikunzhen
 * @details
 * 1. Complete the pid.hpp
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

/**
 * @brief Base parameters of cascaded PID controller
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
struct CascadedPidBaseParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CascadedPidBaseParams only supports float and double");

  T dt = 0;          ///< sampling time, \f$\Delta t\f$, unit: s
  size_t n_pid = 0;  ///< number of pid nodes, \f$n\f$
};

/**
 * @brief Parameters of cascaded PID controller
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
struct CascadedPidParams : public CascadedPidBaseParams<T> {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CascadedPidParams only supports float and double");

  using NodeParamsList = std::vector<PidNodeParams<T>>;

  NodeParamsList node_params_list;  ///< parameters of pid nodes

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * dt: 0
   * n_pid: 0
   *
   * node_params_list:
   *   - kp: 0
   *     ki: 0
   *     kd: 0
   *     dt: 0  # not used
   *     period: 0
   *
   *     en_out_limit: false
   *     # optional, need to be set if en_out_limit is true
   *     out_limit_lb: 0
   *     out_limit_ub: 0
   *
   *     en_deadband: false
   *     # optional, need to be set if en_deadband is true
   *     deadband_lb: 0
   *     deadband_ub: 0
   *
   *     en_trap_int: false
   *
   *     en_anti_windup: false
   *     # optional, need to be set if en_anti_windup is true
   *     anti_windup_lb: 0
   *     anti_windup_ub: 0
   *
   *     en_int_separate: false
   *     # optional, need to be set if en_int_separate is true
   *     int_separate_lb: 0
   *     int_separate_ub: 0
   *
   *     perv_diff_weight: 0
   *
   *     en_td: false
   *     # optional, need to be set if en_td is true
   *     td_cutoff_freq: 0
   *   - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: CascadedPidParams object to store the loaded
   * parameters(the size of `node_params_list` should be equal `n_pid`
   * and `node_params_list[*].dt` will be set to `dt`)
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node,
                                     CascadedPidParams& params)
  {
    params.dt = node["dt"].as<T>();
    params.n_pid = node["n_pid"].as<size_t>();
    params.node_params_list.reserve(params.n_pid);
    params.node_params_list.resize(params.n_pid);

    RU_ASSERT(params.n_pid == node["node_params_list"].size(),
              "The size of node_params_list should be equal to n_pid");

    for (size_t i = 0; i < params.n_pid; ++i) {
      PidNodeParams<T>::LoadParamsFromYamlNode(node["node_params_list"][i],
                                               params.node_params_list[i]);
      params.node_params_list[i].dt = params.dt;
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * dt: 0
   * n_pid: 0
   *
   * node_params_list:
   *   - kp: 0
   *     ki: 0
   *     kd: 0
   *     dt: 0  # not used
   *     period: 0
   *
   *     en_out_limit: false
   *     # optional, need to be set if en_out_limit is true
   *     out_limit_lb: 0
   *     out_limit_ub: 0
   *
   *     en_deadband: false
   *     # optional, need to be set if en_deadband is true
   *     deadband_lb: 0
   *     deadband_ub: 0
   *
   *     en_trap_int: false
   *
   *     en_anti_windup: false
   *     # optional, need to be set if en_anti_windup is true
   *     anti_windup_lb: 0
   *     anti_windup_ub: 0
   *
   *     en_int_separate: false
   *     # optional, need to be set if en_int_separate is true
   *     int_separate_lb: 0
   *     int_separate_ub: 0
   *
   *     perv_diff_weight: 0
   *
   *     en_td: false
   *     # optional, need to be set if en_td is true
   *     td_cutoff_freq: 0
   *   - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return CascadedPidParams object containing the loaded parameters(the size
   * of `node_params_list` should be equal to `n_pid` and
   * `node_params_list[*].dt` will be set to `dt`)
   */
  static CascadedPidParams LoadParamsFromYamlNode(const YAML::Node& node)
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

  T dt = 0;            ///< sampling time, \f$\Delta t\f$, unit: s
  T out_limit_lb = 0;  ///< lower bound of output, \f$u_{min}\f$
  T out_limit_ub = 0;  ///< upper bound of output, \f$u_{max}\f$
  size_t n_pid = 0;    ///< number of pid nodes, \f$n\f$

  bool en_out_limit = false;
};

template <typename T>
struct ParallelPidParams : public ParallelPidBaseParams<T> {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ParallelPidParams only supports float and double");

  using NodeParamsList = std::vector<PidNodeParams<T>>;

  NodeParamsList node_params_list;  ///< parameters of pid nodes

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * dt: 0
   * n_pid: 0
   * en_out_limit: false
   * # optional, can be deleted if en_out_limit is false
   * out_limit_lb: 0
   * out_limit_ub: 0
   *
   * node_params_list:
   *   - kp: 0
   *     ki: 0
   *     kd: 0
   *     dt: 0  # not used
   *     period: 0
   *
   *     en_out_limit: false
   *     # optional, need to be set if en_out_limit is true
   *     out_limit_lb: 0
   *     out_limit_ub: 0
   *
   *     en_deadband: false
   *     # optional, need to be set if en_deadband is true
   *     deadband_lb: 0
   *     deadband_ub: 0
   *
   *     en_trap_int: false
   *
   *     en_anti_windup: false
   *     # optional, need to be set if en_anti_windup is true
   *     anti_windup_lb: 0
   *     anti_windup_ub: 0
   *
   *     en_int_separate: false
   *     # optional, need to be set if en_int_separate is true
   *     int_separate_lb: 0
   *     int_separate_ub: 0
   *
   *     perv_diff_weight: 0
   *
   *     en_td: false
   *     # optional, need to be set if en_td is true
   *     td_cutoff_freq: 0
   *  - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: ParallelPidNodeParams object to store the loaded
   * parameters(the size of the `node_params_list` vector should be equal to
   * `n_pid`, and `node_params_list[*].dt` will be set to `dt`)
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node,
                                     ParallelPidParams& params)
  {
    params.dt = node["dt"].as<T>();
    params.n_pid = node["n_pid"].as<size_t>();
    params.en_out_limit = node["en_out_limit"].as<bool>();
    if (params.en_out_limit) {
      params.out_limit_lb = node["out_limit_lb"].as<T>();
      params.out_limit_ub = node["out_limit_ub"].as<T>();
    }
    params.node_params_list.reserve(params.n_pid);
    params.node_params_list.resize(params.n_pid);

    RU_ASSERT(params.n_pid == node["node_params_list"].size(),
              "The size of node_params_list should be equal to n_pid");

    for (size_t i = 0; i < params.n_pid; ++i) {
      PidNodeParams<T>::LoadParamsFromYamlNode(node["node_params_list"][i],
                                               params.node_params_list[i]);
      params.node_params_list[i].dt = params.dt;
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from the given YAML node. The
   * parameters are expected to be in the following format:
   *
   * ```yaml
   * dt: 0
   * n_pid: 0
   * en_out_limit: false
   * # optional, can be deleted if en_out_limit is false
   * out_limit_lb: 0
   * out_limit_ub: 0
   *
   * node_params_list:
   *   - kp: 0
   *     ki: 0
   *     kd: 0
   *     dt: 0  # not used
   *     period: 0
   *
   *     en_out_limit: false
   *     # optional, need to be set if en_out_limit is true
   *     out_limit_lb: 0
   *     out_limit_ub: 0
   *
   *     en_deadband: false
   *     # optional, need to be set if en_deadband is true
   *     deadband_lb: 0
   *     deadband_ub: 0
   *
   *     en_trap_int: false
   *
   *     en_anti_windup: false
   *     # optional, need to be set if en_anti_windup is true
   *     anti_windup_lb: 0
   *     anti_windup_ub: 0
   *
   *     en_int_separate: false
   *     # optional, need to be set if en_int_separate is true
   *     int_separate_lb: 0
   *     int_separate_ub: 0
   *
   *     perv_diff_weight: 0
   *
   *     en_td: false
   *     # optional, need to be set if en_td is true
   *     td_cutoff_freq: 0
   *  - ...
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return ParallelPidNodeParams object containing the loaded parameters(the
   * size of the `node_params_list` vector should be equal to `n_pid`, and
   * `node_params_list[*].dt` will be set to `dt`)
   */
  static ParallelPidParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    ParallelPidParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Cascaded PID controller class
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
  using Data = typename PidNode<T>::Data;
  using DataList = std::vector<Data>;

  using Ptr = std::shared_ptr<CascadedPid<T>>;
  using ConstPtr = std::shared_ptr<const CascadedPid<T>>;

  /// feedback vector
  using FdbVec = Eigen::VectorX<T>;
  /// feedforward vector
  using FfdVec = Eigen::VectorX<T>;

  /**
   * @brief Constructor of the cascaded PID controller
   * @param params Parameters of the cascaded PID controller(the size of
   * `node_params_list` should be equal to `n_pid`, and `node_params_list[*].dt`
   * will be set to `dt`)
   */
  explicit CascadedPid(const Params& params);
  virtual ~CascadedPid(void) = default;

  /**
   * @brief Calculate the output of the cascaded PID controller
   *
   * @par Pseudo code
   *
   * \f$\qquad u \gets x^d\f$
   *
   * \f$\qquad\mathbf{for}\text{ }i \gets 1 \text{ }\mathbf{to}\text{ } n \text{
   * }\mathbf{do}\f$
   *
   * \f$\qquad\quad u \gets {\rm PidNode}_i\left(u, x_i\right)\f$
   *
   * \f$\qquad\mathbf{end for}\f$
   *
   * \f${\rm PidNode}\f$ is a function that calculates the output of the PID
   * node, defined in @ref PidNode::calc
   *
   * @param[in] ref: Reference value, \f$x^d\f$
   * @param[in] fdb_vec: Feedback vector, \f$\vec{x} = \left[x_1, x_2, \dots,
   * x_{n}\right]^T\f$, size = \f$n\f$
   * @return Output value, \f$u\f$
   */
  T calc(const T& ref, const FdbVec& fdb_vec);

  /**
   * @brief Calculate the output of the cascaded PID controller with feedforward
   *
   * @par Pseudo code
   *
   * \f$\qquad u \gets x^d\f$
   *
   * \f$\qquad\mathbf{for}\text{ }i \gets 1 \text{ }\mathbf{to}\text{ } n \text{
   * }\mathbf{do}\f$
   *
   * \f$\qquad\quad u \gets {\rm PidNode}_i\left(u, x_i, u_i^{ff}\right)\f$
   *
   * \f$\qquad\mathbf{end for}\f$
   *
   * \f${\rm PidNode}\f$ is a function that calculates the output of the PID
   * node, defined in @ref PidNode::calc
   *
   * @param[in] ref: Reference value, \f$x^d\f$
   * @param[in] fdb_vec: Feedback vector, \f$\vec{x} = \left[x_1, x_2, \dots,
   * x_{n}\right]^T\f$, size = \f$n\f$
   * @param[in] ffd_vec: Feedforward vector, \f$\vec{u}^{ff} = \left[u_1^{ff},
   * u_2^{ff}, \dots, u_n^{ff}\right]^T\f$, size = \f$n\f$
   * @return Output value, \f$u\f$
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
   * @param[in] node_params_list: List of PID node parameters, size = \f$n\f$
   * details see @ref PidNode::setParams
   */
  void setNodeParams(const NodeParamsList& node_params_list);

  /**
   * @brief Set the parameters of a specific PID node
   * @param[in] i: Index of the PID node, 0 <= i < \f$n\f$
   * @param[in] node_params: Parameters of the PID node, details see @ref
   * PidNode::setParams
   */
  void setNodeParamsAt(size_t i, const NodeParams& node_params);

  /**
   * @brief Get a specific PID node
   * @param[in] i: Index of the PID node, 0 <= i < \f$n\f$
   * @return PID node
   */
  const PidNode<T>& getNode(size_t i) const { return pid_nodes_[i]; }

 private:
  BaseParams base_params_;

  std::vector<PidNode<T>> pid_nodes_;
};

/**
 * @brief Parallel PID controller class
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
  using Data = typename PidNode<T>::Data;
  using DataList = std::vector<Data>;

  /// reference vector
  using RefVec = Eigen::VectorX<T>;
  /// feedback vector
  using FdbVec = Eigen::VectorX<T>;

  /**
   * @brief Constructor of the parallel PID controller
   * @param params Parameters of the cascaded PID controller(the size of
   * `node_params_list` should be equal to `n_pid`, and `node_params_list[*].dt`
   * will be set to `dt`)
   */
  explicit ParallelPid(const Params& params);
  virtual ~ParallelPid(void) = default;

  /**
   * @brief Calculate the output of the parallel PID controller
   *
   * @par Pseudo code
   *
   * \f$\qquad u \gets 0\f$
   *
   * \f$\qquad\mathbf{for}\text{ }i \gets 1 \text{ }\mathbf{to}\text{ } n \text{
   * }\mathbf{do}\f$
   *
   * \f$\qquad\quad u \gets u + {\rm PidNode}_i\left(x_i^d, x_i\right)\f$
   *
   * \f$\qquad\mathbf{end for}\f$
   *
   * \f$\qquad u \gets \begin{cases}u, & \text{en_out_limit} = false \\
   * \text{Clamp}(u, u_{min}, u_{max}), & \text{else}
   * \end{cases}\f$
   *
   * \f${\rm PidNode}\f$ is a function that calculates the output of the PID
   * node, defined in @ref PidNode::calc
   *
   * @param[in] ref_vec: Reference value, \f$\vec{x}^d = \left[x_1^d, x_2^d,
   * \dots, x_n^d\right]^T\f$, size = \f$n\f$
   * @param[in] fdb_vec: Feedback vector, \f$\vec{x} = \left[x_1, x_2, \dots,
   * x_{n}\right]^T\f$, size = \f$n\f$
   * @return Output value, \f$u\f$
   */
  T calc(const RefVec& ref_vec, const FdbVec& fdb_vec);

  /**
   * @brief Calculate the output of the parallel PID controller with feedforward
   *
   * @par Pseudo code
   *
   * \f$\qquad u \gets u^{ff}\f$
   *
   * \f$\qquad\mathbf{for}\text{ }i \gets 1 \text{ }\mathbf{to}\text{ } n \text{
   * }\mathbf{do}\f$
   *
   * \f$\qquad\quad u \gets u + {\rm PidNode}_i\left(x_i^d, x_i\right)\f$
   *
   * \f$\qquad\mathbf{end for}\f$
   *
   * \f$\qquad u \gets \begin{cases}u, & \text{en_out_limit} = false \\
   * \text{Clamp}(u, u_{min}, u_{max}), & \text{else}
   * \end{cases}\f$
   *
   * \f${\rm PidNode}\f$ is a function that calculates the output of the PID
   * node, defined in @ref PidNode::calc
   *
   * @param[in] ref_vec: Reference value, \f$\vec{x}^d = \left[x_1^d, x_2^d,
   * \dots, x_n^d\right]^T\f$, size = \f$n\f$
   * @param[in] fdb_vec: Feedback vector, \f$\vec{x} = \left[x_1, x_2, \dots,
   * x_{n}\right]^T\f$, size = \f$n\f$
   * @param[in] ffd: Feedforward value, \f$u^{ff}\f$
   * @return Output value, \f$u\f$
   */
  T calc(const RefVec& ref_vec, const FdbVec& fdb_vec, const T& ffd);

  void reset(void)
  {
    for (auto& pid_node : pid_nodes_) {
      pid_node.reset();
    }
  }

  /**
   * @brief Set the base parameters of the PID controller
   * @param[in] base_params: Base parameters of the PID controller(`dt` and
   * `n_pid` will be ignored)
   */
  void setBaseParams(const BaseParams& base_params);
  const BaseParams& getBaseParams(void) const { return base_params_; }

  /**
   * @brief Set the parameters of the PID nodes
   * @param[in] node_params_list: List of PID node parameters, size = \f$n\f$
   * details see @ref PidNode::setParams
   */
  void setNodeParams(const NodeParamsList& node_params_list);

  /**
   * @brief Set the parameters of a specific PID node
   * @param[in] i: Index of the PID node, 0 <= i < \f$n\f$
   * @param[in] node_params: Parameters of the PID node, details see @ref
   * PidNode::setParams
   */
  void setNodeParamsAt(size_t i, const NodeParams& node_params);

  /**
   * @brief Get a specific PID node
   * @param[in] i: Index of the PID node, 0 <= i < \f$n\f$
   * @return PID node
   */
  const PidNode<T>& getNode(size_t i) const { return pid_nodes_[i]; }

 private:
  BaseParams base_params_;

  std::vector<PidNode<T>> pid_nodes_;
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
