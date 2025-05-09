/**
 *******************************************************************************
 * @file pid.cpp
 * @brief Cascaded PID and parallel PID controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-09
 * @author Caikunzhen
 * @details
 * 1. Complete the pid.cpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "robot_utils/controller/pid.hpp"
/* Private macro -------------------------------------------------------------*/

namespace robot_utils
{
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/

template <typename T>
CascadedPid<T>::CascadedPid(const Params& params) : base_params_(params)
{
  RU_ASSERT(params.n_pid > 0,
            "The number of pid nodes should be greater than 0");
  RU_ASSERT(params.n_pid == params.node_params_list.size(),
            "The size of node_params_list should be equal to n_pid");

  pid_nodes_.reserve(params.n_pid);
  for (size_t i = 0; i < params.n_pid; ++i) {
    NodeParams node_params = params.node_params_list[i];
    node_params.dt = params.dt;
    pid_nodes_.emplace_back(node_params);
  }

  reset();
}

template <typename T>
T CascadedPid<T>::calc(const T& ref, const FdbVec& fdb_vec)
{
  RU_ASSERT(
      fdb_vec.size() == base_params_.n_pid,
      "The size of feedback vector should be equal to the number of pid nodes");

  T out = ref;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out = pid_nodes_[i].calc(out, fdb_vec[i]);
  }

  return out;
}

template <typename T>
T CascadedPid<T>::calc(const T& ref, const FdbVec& fdb_vec,
                       const FfdVec& ffd_vec)
{
  RU_ASSERT(
      fdb_vec.size() == base_params_.n_pid,
      "The size of feedback vector should be equal to the number of pid nodes");
  RU_ASSERT(ffd_vec.size() == base_params_.n_pid,
            "The size of feedforward vector should be equal to the number "
            "of pid nodes");

  T out = ref;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out = pid_nodes_[i].calc(out, fdb_vec[i], ffd_vec[i]);
  }

  return out;
}

template <typename T>
void CascadedPid<T>::setNodeParams(const NodeParamsList& node_params_list)
{
  RU_ASSERT(node_params_list.size() == base_params_.n_pid,
            "The size of node_params_list should be equal to the number of "
            "pid nodes");

  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    pid_nodes_[i].setParams(node_params_list[i]);
  }
}

template <typename T>
void CascadedPid<T>::setNodeParamsAt(size_t i, const NodeParams& node_params)
{
  RU_ASSERT(i < base_params_.n_pid,
            "The index of node_params should be less than n_pid");

  pid_nodes_[i].setParams(node_params);
}

template <typename T>
ParallelPid<T>::ParallelPid(const Params& params) : base_params_(params)
{
  RU_ASSERT(params.n_pid > 0,
            "The number of pid nodes should be greater than 0");

  pid_nodes_.reserve(params.n_pid);
  for (size_t i = 0; i < params.n_pid; ++i) {
    NodeParams node_params = params.node_params_list[i];
    node_params.dt = params.dt;
    pid_nodes_.emplace_back(node_params);
  }

  reset();
}

template <typename T>
T ParallelPid<T>::calc(const RefVec& ref_vec, const FdbVec& fdb_vec)
{
  RU_ASSERT(ref_vec.size() == base_params_.n_pid,
            "The size of reference vector should be equal to the number of "
            "pid nodes");
  RU_ASSERT(fdb_vec.size() == base_params_.n_pid,
            "The size of feedback vector should be equal to the number of "
            "pid nodes");

  T out = 0;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out += pid_nodes_[i].calc(ref_vec[i], fdb_vec[i]);
  }

  if (base_params_.en_out_limit) {
    out = std::clamp(out, base_params_.out_limit_lb, base_params_.out_limit_ub);
  }

  return out;
}

template <typename T>
T ParallelPid<T>::calc(const RefVec& ref_vec, const FdbVec& fdb_vec,
                       const T& ffd)
{
  RU_ASSERT(ref_vec.size() == base_params_.n_pid,
            "The size of reference vector should be equal to the number of "
            "pid nodes");
  RU_ASSERT(fdb_vec.size() == base_params_.n_pid,
            "The size of feedback vector should be equal to the number of "
            "pid nodes");

  T out = ffd;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out += pid_nodes_[i].calc(ref_vec[i], fdb_vec[i]);
  }

  if (base_params_.en_out_limit) {
    out = std::clamp(out, base_params_.out_limit_lb, base_params_.out_limit_ub);
  }

  return out;
}

template <typename T>
void ParallelPid<T>::setBaseParams(const BaseParams& base_params)
{
  T dt = base_params.dt;
  T n_pid = base_params.n_pid;
  base_params_ = base_params;
  base_params_.dt = dt;
  base_params_.n_pid = n_pid;
}

template <typename T>
void ParallelPid<T>::setNodeParams(const NodeParamsList& node_params_list)
{
  RU_ASSERT(node_params_list.size() == base_params_.n_pid,
            "The size of node_params_list should be equal to the number of "
            "pid nodes");

  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    pid_nodes_[i].setParams(node_params_list[i]);
  }
}

template <typename T>
void ParallelPid<T>::setNodeParamsAt(size_t i, const NodeParams& node_params)
{
  RU_ASSERT(i < base_params_.n_pid,
            "The index of node_params should be less than n_pid");

  pid_nodes_[i].setParams(node_params);
}

template class CascadedPid<float>;
template class CascadedPid<double>;
template class ParallelPid<float>;
template class ParallelPid<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
