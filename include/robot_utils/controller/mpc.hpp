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

/**
 * @brief Parameters of MPC controller
 */
struct MpcParams {
  using real_t = qpOASES::real_t;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;

  size_t n = 0;        ///< number of states, \f$n\f$
  size_t m = 0;        ///< number of inputs, \f$m\f$
  size_t horizon = 0;  ///< prediction horizon, \f$N\f$
  /// maximum number of iterations for QP solver
  qpOASES::int_t max_iter = 100;

  /// state matrix, \f$A \in \mathbb{R}^{n \times n}\f$
  Eigen::MatrixX<real_t> A;
  /// input matrix, \f$B \in \mathbb{R}^{n \times m}\f$
  Eigen::MatrixX<real_t> B;
  DiagMatX Q;  ///< state cost matrix, \f$Q \in \mathbb{R}^{n \times n}\f$
  DiagMatX P;  ///< terminal cost matrix, \f$P \in \mathbb{R}^{n \times n}\f$
  DiagMatX R;  ///< input cost matrix, \f$R \in \mathbb{R}^{m \times m}\f$

  /// state lower bound, \f$x_{min} \in \mathbb{R}^n\f$
  Eigen::VectorX<real_t> x_min;
  /// state upper bound, \f$x_{max} \in \mathbb{R}^n\f$
  Eigen::VectorX<real_t> x_max;
  /// input lower bound, \f$u_{min} \in \mathbb{R}^m\f$
  Eigen::VectorX<real_t> u_min;
  /// input upper bound, \f$u_{max} \in \mathbb{R}^m\f$
  Eigen::VectorX<real_t> u_max;

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
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * x_min: [<x_min values>] # size (1, n)
   * x_max: [<x_max values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: MPC parameters
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
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * x_min: [<x_min values>] # size (1, n)
   * x_max: [<x_max values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return MPC parameters
   */
  static MpcParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    MpcParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Model Predictive Control (MPC) controller
 *
 * Problem formulation:
 *
 * \f[
 * \underset{u_{0:N-1}^*}{\min} u_0^T R u_0 + \sum_{i=1}^{N-1} *
 * \left(\left(x_i^d - x_i\right)^T Q
 * \left(x_i^d - x_i\right) + u_i^T R u_i\right) + \left(x_N^d - x_N\right)^T P
 * \left(x_i^d - x_i\right)
 * \f]
 *
 * s.t.
 *
 * \f[
 * x_{k+1} = Ax_k + Bu_k
 * \f]
 *
 * \f[
 * x_{min} \leq x_i \leq x_{max}, i = 1, \ldots, N
 * \f]
 *
 * \f[
 * u_{min} \leq u_i \leq u_{max}, i = 0, \ldots, N - 1
 * \f]
 */
class Mpc
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = MpcParams;
  using Ptr = std::shared_ptr<Mpc>;
  using ConstPtr = std::shared_ptr<const Mpc>;

  using real_t = qpOASES::real_t;
  /// state vector, \f$x \in \mathbb{R}^n\f$
  using StateVec = Eigen::VectorX<real_t>;
  /// input vector, \f$u \in \mathbb{R}^m\f$
  using InputVec = Eigen::VectorX<real_t>;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;
  using CtrlSeq = std::vector<InputVec>;
  using StateSeq = std::vector<StateVec>;

  explicit Mpc(const Params& params);
  virtual ~Mpc(void) = default;

  /**
   * @brief Solve the MPC problem
   *
   * This function solves the MPC problem using the QP solver. It sets the
   * initial state and the reference state, and then calls the QP solver to
   * solve the problem. The function returns true if the problem is solved
   * successfully, and false otherwise.
   *
   * @param[in] x_ref_seq: sequence of reference state vector, \f$x_{1:N}^d =
   * \left[x_1^d, x_2^d, \ldots, x_N^d\right]\f$
   * @param[in] x0: initial state vector, \f$x_0 \in \mathbb{R}^n\f$
   * @param[in] force_init: whether to force re-initialization of the QP solver
   * @return true if the problem is solved successfully, false otherwise
   */
  bool solve(const StateSeq& x_ref_seq, const StateVec& x0,
             bool force_init = false);

  /**
   * @brief Get the input vector
   * @param[out] u: input vector, \f$u_f \in \mathbb{R}^m\f$
   * @param[in] forward_steps: number of forward steps, \f$f\f$, when it is
   * greater than horizon - 1, it will be set to horizon - 1
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  void getCtrl(InputVec& u, size_t forward_steps = 0) const;

  /**
   * @brief Get the input vector
   * @param[in] forward_steps: number of forward steps, \f$f\f$, when it is
   * greater than horizon - 1, it will be set to horizon - 1
   * @return input vector, \f$u_f \in \mathbb{R}^m\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  InputVec getCtrl(size_t forward_steps = 0) const
  {
    InputVec u;
    getCtrl(u, forward_steps);
    return u;
  }

  /**
   * @brief Get the control sequence
   * @param[out] u_seq: control sequence, \f$u_{0:N-1} =
   * \left[u_0, u_1, \ldots, u_{N-1}\right]\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  void getCtrlSeq(CtrlSeq& u_seq) const;

  /**
   * @brief Get the control sequence
   * @return control sequence, \f$u_{0:N-1} =
   * \left[u_0, u_1, \ldots, u_{N-1}\right]\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  CtrlSeq getCtrlSeq(void) const
  {
    CtrlSeq u_seq;
    getCtrlSeq(u_seq);
    return u_seq;
  }

  /**
   * @brief Get the predicted state sequence
   * @param[out] x_seq: state sequence, \f$x_{1:N} =
   * \left[x_1, x_2, \ldots, x_{N}\right]\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  void getPredStateSeq(StateSeq& x_seq) const;

  /**
   * @brief Get the predicted state sequence
   * @return state sequence, \f$x_{1:N} =
   * \left[x_1, x_2, \ldots, x_{N}\right]\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined.
   */
  StateSeq getPredStateSeq(void) const
  {
    StateSeq x_seq;
    getPredStateSeq(x_seq);
    return x_seq;
  }

  /**
   * @brief Set the parameters of the MPC controller
   * @param params MPC parameters
   * @note params.n, params.m and params.horizon will be ignored.
   */
  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

 private:
  void calcSBar(void);

  void calcTBar(void);

  void calcQBar(void);

  void calcRBar(void);

  void calcH(void);

  void calcG(const StateSeq& x_ref_seq);

  void calcA(void) { A_ = S_bar_; }

  void calcLbUb(void);

  void calcLbAUbA(void);

  Params params_;

  Eigen::VectorX<real_t> x0_;  ///< initial state, \f$x_0 \in \mathbb{R}^n\f$
  /// input transition matrix, \f$\bar S \in \mathbb{R}^{nN \times mN}\f$
  Eigen::MatrixX<real_t> S_bar_;
  /// init state transition matrix, \f$\bar T \in \mathbb{R}^{nN \times n}\f$
  Eigen::MatrixX<real_t> T_bar_;
  /// state cost matrix, \f$\bar Q \in \mathbb{R}^{nN \times nN}\f$
  DiagMatX Q_bar_;
  /// input cost matrix, \f$\bar R \in \mathbb{R}^{mN \times mN}\f$
  DiagMatX R_bar_;

  /// Hessian matrix, \f$^{qp}H \in \mathbb{R}^{mN \times mN}\f$
  Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_;
  /// gradient vector, \f$^{qp}g \in \mathbb{R}^{mN}\f$
  Eigen::VectorX<real_t> g_;
  /// lower bound vector, \f$^{qp}U_{min} \in \mathbb{R}^{mN}\f$
  Eigen::VectorX<real_t> lb_;
  /// upper bound vector, \f$^{qp}U_{max} \in \mathbb{R}^{mN}\f$
  Eigen::VectorX<real_t> ub_;
  /// constraint matrix, \f$^{qp}A \in \mathbb{R}^{nN \times mN}\f$
  Eigen::Matrix<real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_;
  /// lower bound vector for constraints, \f$^{qp}b_{min} \in \mathbb{R}^{nN}\f$
  Eigen::VectorX<real_t> lbA_;
  /// upper bound vector for constraints, \f$^{qp}b_{max} \in \mathbb{R}^{nN}\f$
  Eigen::VectorX<real_t> ubA_;
  /**
   * input vector, \f$^{qp}U = \left[u_0^T, u_1^T, \ldots,
   * u_{N-1}^T\right]^T \in \mathbb{R}^{mN}\f$
   */
  Eigen::VectorX<real_t> U_;

  qpOASES::QProblem qp_;
  bool can_hot_start_ = false;
  bool solved_ = false;  ///< whether the QP problem is solved
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* HAS_QPOASES */
#endif /* ROBOT_UTILS_CONTROLLER_MPC_HPP_ */
