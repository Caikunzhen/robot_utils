/**
 *******************************************************************************
 * @file      : pid.cpp
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
  PARAM_ASSERT(params.ctrl_rate > 0,
               "The control rate should be greater than 0");
  PARAM_ASSERT(params.n_pid > 0,
               "The number of pid nodes should be greater than 0");

  pid_nodes_.reserve(params.n_pid);
  for (size_t i = 0; i < params.n_pid; ++i) {
    NodeParams node_params = params.node_params[i];
    node_params.ctrl_rate = params.ctrl_rate;
    pid_nodes_.emplace_back(node_params);
  }

  reset();
}

template <typename T>
T CascadedPid<T>::calc(const T& ref, const FdbVec& fdb_vec)
{
  PARAM_ASSERT(
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
  PARAM_ASSERT(
      fdb_vec.size() == base_params_.n_pid,
      "The size of feedback vector should be equal to the number of pid nodes");
  PARAM_ASSERT(ffd_vec.size() == base_params_.n_pid,
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
  PARAM_ASSERT(node_params_list.size() == base_params_.n_pid,
               "The size of node_params_list should be equal to the number of "
               "pid nodes");

  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    NodeParams node_params = node_params_list[i];
    node_params.ctrl_rate = base_params_.ctrl_rate;
    pid_nodes_[i].setParams(node_params);
  }
}

template <typename T>
void CascadedPid<T>::setNodeParamsAt(size_t i, const NodeParams& node_params)
{
  NodeParams node_params_copy = node_params;
  node_params_copy.ctrl_rate = base_params_.ctrl_rate;
  pid_nodes_[i].setParams(node_params_copy);
}

template <typename T>
ParallelPid<T>::ParallelPid(const Params& params) : base_params_(params)
{
  PARAM_ASSERT(params.ctrl_rate > 0,
               "The control rate should be greater than 0");
  PARAM_ASSERT(params.n_pid > 0,
               "The number of pid nodes should be greater than 0");

  pid_nodes_.reserve(params.n_pid);
  for (size_t i = 0; i < params.n_pid; ++i) {
    NodeParams node_params = params.node_params[i];
    node_params.ctrl_rate = params.ctrl_rate;
    pid_nodes_.emplace_back(node_params);
  }

  reset();
}

template <typename T>
T ParallelPid<T>::calc(const RefVec& ref_vec, const FdbVec& fdb_vec)
{
  PARAM_ASSERT(ref_vec.size() == base_params_.n_pid,
               "The size of reference vector should be equal to the number of "
               "pid nodes");
  PARAM_ASSERT(fdb_vec.size() == base_params_.n_pid,
               "The size of feedback vector should be equal to the number of "
               "pid nodes");

  T out = 0;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out += pid_nodes_[i].calc(ref_vec[i], fdb_vec[i]);
  }

  return out;
}

template <typename T>
T ParallelPid<T>::calc(const RefVec& ref_vec, const FdbVec& fdb_vec,
                       const FfdVec& ffd_vec)
{
  PARAM_ASSERT(ref_vec.size() == base_params_.n_pid,
               "The size of reference vector should be equal to the number of "
               "pid nodes");
  PARAM_ASSERT(fdb_vec.size() == base_params_.n_pid,
               "The size of feedback vector should be equal to the number of "
               "pid nodes");
  PARAM_ASSERT(ffd_vec.size() == base_params_.n_pid,
               "The size of feedforward vector should be equal to the number "
               "of pid nodes");

  T out = 0;
  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    out += pid_nodes_[i].calc(ref_vec[i], fdb_vec[i], ffd_vec[i]);
  }

  return out;
}

template <typename T>
void ParallelPid<T>::setNodeParams(const NodeParamsList& node_params_list)
{
  PARAM_ASSERT(node_params_list.size() == base_params_.n_pid,
               "The size of node_params_list should be equal to the number of "
               "pid nodes");

  for (size_t i = 0; i < base_params_.n_pid; ++i) {
    NodeParams node_params = node_params_list[i];
    node_params.ctrl_rate = base_params_.ctrl_rate;
    pid_nodes_[i].setParams(node_params);
  }
}

template <typename T>
void ParallelPid<T>::setNodeParamsAt(size_t i, const NodeParams& node_params)
{
  NodeParams node_params_copy = node_params;
  node_params_copy.ctrl_rate = base_params_.ctrl_rate;
  pid_nodes_[i].setParams(node_params_copy);
}

template class CascadedPid<float>;
template class CascadedPid<double>;
template class ParallelPid<float>;
template class ParallelPid<double>;
/* Private function definitions ----------------------------------------------*/
}  // namespace robot_utils
