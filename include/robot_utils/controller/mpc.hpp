/**
 *******************************************************************************
 * @file      : mpc.hpp
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
#ifndef ROBOT_UTILS_CONTROLLER_MPC_HPP_
#define ROBOT_UTILS_CONTROLLER_MPC_HPP_
#ifdef HAS_QPOASES

/* Includes ------------------------------------------------------------------*/
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <qpOASES.hpp>
#include <type_traits>
#include <vector>

#include "robot_utils/core/assert.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct MpcParams {
  using real_t = qpOASES::real_t;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;

  size_t n = 0;        //<! number of states
  size_t m = 0;        //<! number of inputs
  size_t horizon = 0;  //<! prediction horizon
  //<! maximum number of iterations for QP solver
  qpOASES::int_t max_iter = 100;

  Eigen::MatrixX<real_t> A;  //<! state matrix
  Eigen::MatrixX<real_t> B;  //<! input matrix
  DiagMatX Q;                //<! state cost matrix
  DiagMatX R;                //<! input cost matrix
  DiagMatX P;                //<! terminal cost matrix

  Eigen::VectorX<real_t> x_min_;  //<! state lower bound
  Eigen::VectorX<real_t> x_max_;  //<! state upper bound
  Eigen::VectorX<real_t> u_min_;  //<! input lower bound
  Eigen::VectorX<real_t> u_max_;  //<! input upper bound

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * n: <number of states>
   * m: <number of inputs>
   * horizon: <prediction horizon>
   * max_iter: <maximum number of iterations>
   * A: [<A matrix values>] # optional, size (1, n * n), row-major order
   * B: [<B matrix values>] # optional, size (1, n * m), row-major order
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * x_min: [<x_min values>] # size (1, n)
   * x_max: [<x_max values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: MPC parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node, MpcParams& params);

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * n: <number of states>
   * m: <number of inputs>
   * horizon: <prediction horizon>
   * max_iter: <maximum number of iterations>
   * A: [<A matrix values>] # optional, size (1, n * n), row-major order
   * B: [<B matrix values>] # optional, size (1, n * m), row-major order
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * x_min: [<x_min values>] # size (1, n)
   * x_max: [<x_max values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return MPC parameters
   * @note None
   */
  static MpcParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    MpcParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

class Mpc
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = MpcParams;
  using Ptr = std::shared_ptr<Mpc>;
  using ConstPtr = std::shared_ptr<const Mpc>;

  using real_t = qpOASES::real_t;
  //<! expected state vector, size (n * horizon, 1)
  using RefStateVec = Eigen::VectorX<real_t>;
  using StateVec = Eigen::VectorX<real_t>;  //<! state vector, size (n, 1)
  using InputVec = Eigen::VectorX<real_t>;  //<! input vector, size (m, 1)
  using DiagMatX =
      Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;  //<! diagonal matrix
  //<! control sequence, size (horizon, m)
  using CtrlSeq = std::vector<InputVec>;
  //<! state sequence, size (horizon, n)
  using StateSeq = std::vector<StateVec>;

  explicit Mpc(const Params& params) { setParams(params); }
  ~Mpc(void) = default;

  /**
   * @brief Solve the MPC problem
   *
   * This function solves the MPC problem using the QP solver. It sets the
   * initial state and the reference state, and then calls the QP solver to
   * solve the problem. The function returns true if the problem is solved
   * successfully, and false otherwise.
   *
   * @param[in] x_ref: reference state vector, size (n * horizon, 1)
   * @param[in] x0: initial state vector, size (n, 1)
   * @param[in] force_init: whether to force re-initialization of the QP solver
   * @return true if the problem is solved successfully, false otherwise
   * @note None
   */
  bool solve(const RefStateVec& x_ref, const StateVec& x0,
             bool force_init = false);

  /**
   * @brief Get the input vector
   * @param[out] u: input vector, size (m, 1)
   * @param[in] forward_steps: number of forward steps, when it is greater than
   * horizon - 1, it will be set to horizon - 1
   * @return None
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  void getCtrl(InputVec& u, size_t forward_steps = 0) const;

  /**
   * @brief Get the input vector
   * @param[in] forward_steps: number of forward steps, when it is greater than
   * horizon - 1, it will be set to horizon - 1
   * @return Input vector, size (m, 1)
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  InputVec getCtrl(size_t forward_steps = 0) const
  {
    InputVec u;
    getCtrl(u, forward_steps);
    return u;
  }

  /**
   * @brief Get the control sequence
   * @param[out] u_seq: control sequence, size (horizon, m)
   * @return None
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  void getCtrlSeq(CtrlSeq& u_seq) const;

  /**
   * @brief Get the control sequence
   * @return Control sequence, size (horizon, m)
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  CtrlSeq getCtrlSeq(void) const
  {
    CtrlSeq u_seq;
    getCtrlSeq(u_seq);
    return u_seq;
  }

  /**
   * @brief Get the state sequence
   * @param[out] x_seq: state sequence, size (horizon, n)
   * @return None
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  void getPredStateSeq(StateSeq& x_seq) const;

  /**
   * @brief Get the state sequence
   * @return State sequence, size (horizon, n)
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined
   */
  StateSeq getPredStateSeq(void) const
  {
    StateSeq x_seq;
    getPredStateSeq(x_seq);
    return x_seq;
  }

  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

 private:
  void calcSBar(void);

  void calcTBar(void);

  void calcQBar(void);

  void calcRBar(void);

  void calcH(void);

  void calcG(const RefStateVec& x_ref);

  void calcA(void) { A_ = S_bar_; }

  void calcLbUb(void);

  void calcLbAUbA(void);

  Params params_;

  Eigen::VectorX<real_t> x0_;  //<! initial state, size (n, 1)
  Eigen::MatrixX<real_t>
      S_bar_;  //<! input transition matrix, size (n * horizon, m * horizon)
  Eigen::MatrixX<real_t>
      T_bar_;  //<! init state transition matrix, size (n * horizon, n)
  /*<! state cost matrix(with terminal cost matrix),
   * size (n * horizon, n * horizon) */
  DiagMatX Q_bar_;
  DiagMatX R_bar_;  //<! input cost matrix, size (m * horizon, m * horizon)

  //<! Hessian matrix, size (m * horizon, m * horizon)
  Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
  Eigen::VectorX<real_t> g_;   //<! gradient vector, size (m * horizon, 1)
  Eigen::VectorX<real_t> lb_;  //<! lower bound vector, size (m * horizon, 1)
  Eigen::VectorX<real_t> ub_;  //<! upper bound vector, size (m * horizon, 1)
  //<! constraint matrix, size (2 * n * horizon, m * horizon)
  Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
  //<! lower bound vector for constraints, size (2 * n * horizon, 1)
  Eigen::VectorX<real_t> lbA_;
  //<! upper bound vector for constraints, size (2 * n * horizon, 1)
  Eigen::VectorX<real_t> ubA_;
  Eigen::VectorX<real_t> U_bar_;  //<! input vector, size (m * horizon, 1)

  qpOASES::QProblem qp_;  //<! QP solver
  bool can_hot_start_ = false;
  bool solved_ = false;  //<! whether the QP problem is solved
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace robot_utils

#endif /* HAS_QPOASES */
#endif /* ROBOT_UTILS_CONTROLLER_MPC_HPP_ */
