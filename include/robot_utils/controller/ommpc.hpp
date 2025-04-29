/**
 *******************************************************************************
 * @file      : ommpc.hpp
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
#ifndef ROBOT_UTILS_CONTROLLER_OMMPC_HPP_
#define ROBOT_UTILS_CONTROLLER_OMMPC_HPP_
#ifdef HAS_QPOASES

/* Includes ------------------------------------------------------------------*/
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cstddef>
#include <functional>
#include <memory>
#include <qpOASES.hpp>
#include <type_traits>
#include <vector>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/geometry/manifold.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

struct OmmpcParams {
  using real_t = qpOASES::real_t;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;
  using DynFunc = std::function<Eigen::VectorX<real_t>(
      const CompoundManifold<real_t>&, const Eigen::VectorX<real_t>&)>;
  using SysSpecificJacobianFunc = std::function<Eigen::MatrixX<real_t>(
      const CompoundManifold<real_t>&, const Eigen::VectorX<real_t>&)>;

  size_t n = 0;        //<! number of states
  size_t m = 0;        //<! number of inputs
  size_t horizon = 0;  //<! prediction horizon
  //<! maximum number of iterations for QP solver
  qpOASES::int_t max_iter = 100;
  real_t dt = 0;  //<! time step

  DynFunc f;  //<! dynamic function
  //<! dynamic function Jacobian with respect to dx
  SysSpecificJacobianFunc f_dx;
  //<! dynamic function Jacobian with respect to du
  SysSpecificJacobianFunc f_du;
  DiagMatX Q;  //<! state cost matrix
  DiagMatX R;  //<! input cost matrix
  DiagMatX P;  //<! terminal cost matrix

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
   * dt: <time step>
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: OMMPC parameters
   * @return None
   * @note None
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node,
                                     OmmpcParams& params);

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
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * u_min: [<u_min values>] # size (1, m)
   * u_max: [<u_max values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return OMMPC parameters
   * @note None
   */
  static OmmpcParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    OmmpcParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

class Ommpc
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = OmmpcParams;
  using Ptr = std::shared_ptr<Ommpc>;
  using ConstPtr = std::shared_ptr<const Ommpc>;

  using real_t = qpOASES::real_t;
  using StateVec = CompoundManifold<real_t>;  //<! state vector, size (n, 1)
  using InputVec = Eigen::VectorX<real_t>;    //<! input vector, size (m, 1)
  using DiagMatX =
      Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;  //<! diagonal matrix
  //<! control sequence, size (horizon, m)
  using CtrlSeq = std::vector<InputVec>;
  //<! state sequence, size (horizon, n)
  using StateSeq = std::vector<StateVec>;

  explicit Ommpc(const Params& params);
  virtual ~Ommpc(void) = default;

  /**
   * @brief Solve the OMMPC problem
   *
   * This function solves the OMMPC problem using the QP solver. It sets the
   * initial state and the reference state, and then calls the QP solver to
   * solve the problem. The function returns true if the problem is solved
   * successfully, and false otherwise.
   *
   * @param[in] x_ref_seq: sequence of reference state vector, size horizon *
   * (n, 1)
   * @param[in] u_ref_seq: sequence of reference input vector, size horizon *
   * (m, 1)
   * @param[in] x0: initial state vector, size (n, 1)
   * @param[in] force_init: whether to force re-initialization of the QP solver
   * @return true if the problem is solved successfully, false otherwise
   * @note None
   */
  bool solve(const StateSeq& x_ref_seq, const CtrlSeq& u_ref_seq,
             const StateVec& x0);

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
  void calcQBar(void);

  void calcRBar(void);

  void getManifoldSpecificJacobian(const StateVec& x_ref,
                                   const ManifoldBase<real_t>::HomeSpace& delta,
                                   Eigen::MatrixX<real_t>& G_x,
                                   Eigen::MatrixX<real_t>& G_f);

  void getPrimManifoldSpecificJacobian(
      const ManifoldBase<real_t>& prim_m,
      const ManifoldBase<real_t>::HomeSpace& delta, Eigen::MatrixX<real_t>& G_x,
      Eigen::MatrixX<real_t>& G_f);

  void getJacobian(const StateVec& x_ref, const InputVec& u_ref,
                   Eigen::MatrixX<real_t>& F_x, Eigen::MatrixX<real_t>& F_u);

  Params params_;

  Eigen::VectorX<real_t> dx0_;  //<! initial state perturbation, size (n, 1)
  StateSeq x_ref_seq_;          //<! reference state sequence, size (horizon, n)
  CtrlSeq u_ref_seq_;           //<! reference input sequence, size (horizon, m)
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
  Eigen::VectorX<real_t> dU_bar_;  //<! input vector, size (m * horizon, 1)

  qpOASES::QProblem qp_;  //<! QP solver
  bool solved_ = false;   //<! whether the QP problem is solved
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace robot_utils

#endif /* HAS_QPOASES */
#endif /* ROBOT_UTILS_CONTROLLER_OMMPC_HPP_ */
