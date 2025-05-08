/**
 *******************************************************************************
 * @file ommpc.hpp
 * @brief This file provides the OMMPC class for robot_utils, it is based on the
 * paper:
 *
 * G. Lu, W. Xu and F. Zhang, "On-Manifold Model Predictive Control for
 * Trajectory Tracking on Robotic Systems," in IEEE Transactions on Industrial
 * Electronics, vol. 70, no. 9, pp. 9192-9202, Sept. 2023,
 * doi: 10.1109/TIE.2022.3212397.
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

/**
 * @brief Parameters of OMMPC controller
 */
struct OmmpcParams {
  using real_t = qpOASES::real_t;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;
  /**
   * @brief Dynamic function type, \f$f\left(\mathrm{x}, u\right)\f$
   * @param[in] x: state vector, \f$\mathrm{x} \in \mathcal{M}\f$, demension
   * \f$n\f$
   * @param[in] u: input vector, \f$u \in \mathbb{R}^m\f$
   * @return \f$f\left(\mathrm{x}, u\right)\f$
   */
  using DynFunc = std::function<Eigen::VectorX<real_t>(
      const CompoundManifold<real_t>&, const Eigen::VectorX<real_t>&)>;
  using SysSpecificJacobianFunc = std::function<Eigen::MatrixX<real_t>(
      const CompoundManifold<real_t>&, const Eigen::VectorX<real_t>&)>;

  size_t n = 0;        ///< demension of manifold, \f$n\f$
  size_t m = 0;        ///< number of inputs, \f$m\f$
  size_t horizon = 0;  ///< prediction horizon, \f$N\f$
  /// maximum number of iterations for QP solver
  qpOASES::int_t max_iter = 100;
  real_t dt = 0;  ///< time step, \f$\Delta t\f$

  DynFunc f;
  /**
   * @brief \f$\frac{\partial f\left(\mathrm{x}, u\boxplus\delta
   * u\right)}{\partial \delta u} |_{\delta u = 0}\f$
   * @param[in] x: state vector, \f$\mathrm{x} \in \mathcal{M}\f$, demension
   * \f$n\f$
   * @param[in] u: input vector, \f$u \in \mathbb{R}^m\f$
   * @return Jacobian matrix, \f$\frac{\partial f\left(\mathrm{x},
   * u\boxplus\delta u\right)}{\partial \delta u} |_{\delta u = 0} \in
   * \mathbb{R}^{n \times n}\f$
   */
  SysSpecificJacobianFunc df_dx;
  /**
   * @brief \f$\frac{\partial f\left(\mathrm{x}, u\boxplus\delta
   * u\right)}{\partial
   * \delta u} |_{\delta u = 0}\f$
   * @param[in] x: state vector, \f$\mathrm{x} \in \mathcal{M}\f$, demension
   * \f$n\f$
   * @param[in] u: input vector, \f$u \in \mathbb{R}^m\f$
   * @return Jacobian matrix, \f$\frac{\partial
   * f\left(\mathrm{x}, u\boxplus\delta u\right)}{\partial
   * \delta u} |_{\delta u = 0} \in \mathbb{R}^{n \times m}\f$
   */
  SysSpecificJacobianFunc df_du;
  DiagMatX Q;  ///< state cost matrix, \f$Q \in \mathbb{R}^{n \times n}\f$
  DiagMatX P;  ///< terminal cost matrix, \f$P \in \mathbb{R}^{n \times n}\f$
  DiagMatX R;  ///< input cost matrix, \f$R \in \mathbb{R}^{m \times m}\f$

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
   * dt: <time step>
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
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
   * P_diag: [<P matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
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

/**
 * @brief On-Manifold Model Predictive Control (OMMPC) controller
 */
class Ommpc
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = OmmpcParams;
  using Ptr = std::shared_ptr<Ommpc>;
  using ConstPtr = std::shared_ptr<const Ommpc>;

  using real_t = qpOASES::real_t;
  /// state vector, \f$\mathrm{x} \in \mathcal{M}\f$, demension \f$n\f$
  using StateVec = CompoundManifold<real_t>;
  /// input vector, \f$u \in \mathbb{R}^m\f$
  using InputVec = Eigen::VectorX<real_t>;
  using DiagMatX = Eigen::DiagonalMatrix<real_t, Eigen::Dynamic>;
  using CtrlSeq = std::vector<InputVec>;
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
   * @param[in] x_ref_seq: sequence of reference state vector,
   * \f$\mathrm{x}_{1:N}^d = \left[\mathrm{x}_1^d, \mathrm{x}_2^d, \ldots,
   * \mathrm{x}_N^d\right]\f$
   * @param[in] u_ref_seq: sequence of reference input vector, \f$u_{0:N-1}^d =
   * \left[u_0^d, u_1^d, \ldots, u_{N-1}^d\right]\f$
   * @param[in] x0: initial state vector, \f$\mathrm{x}_0 \in \mathcal{M}\f$,
   * demension \f$n\f$
   * @param[in] force_init: whether to force re-initialization of the QP solver
   * @return true if the problem is solved successfully, false otherwise
   */
  bool solve(const StateSeq& x_ref_seq, const CtrlSeq& u_ref_seq,
             const StateVec& x0);

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
   * @note Call `solve` method first and it must return true, otherwise
   * operation will be undefined.
   */
  CtrlSeq getCtrlSeq(void) const
  {
    CtrlSeq u_seq;
    getCtrlSeq(u_seq);
    return u_seq;
  }

  /**
   * @brief Get the state sequence
   * @param[out] x_seq: state sequence, \f$x_{1:N} =
   * \left[x_1, x_2, \ldots, x_{N}\right]\f$
   * @note Call @ref solve method first and it must return true, otherwise
   * operation will be undefined
   */
  void getPredStateSeq(StateSeq& x_seq) const;

  /**
   * @brief Get the state sequence
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
   * @brief Set the parameters of the OMMPC controller
   * @param params OMMPC parameters
   * @note params.n, params.m, params.horizon and params.dt will be ignored.
   */
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

  /// initial state, \f$\delta\mathrm{x}_0 \in \mathbb{R}^n\f$
  Eigen::VectorX<real_t> dx0_;
  /**
   * referece state sequence, \f$\mathrm{x}_{0:N}^d =
   * \left[\mathrm{x}_0^d, \mathrm{x}_1^d, \ldots, \mathrm{x}_N^d\right]\f$
   */
  StateSeq x_ref_seq_;
  /**
   * reference input sequence, \f$u_{0:N-1}^d =
   * \left[u_0^d, u_1^d, \ldots, u_{N-1}^d\right]\f$
   */
  CtrlSeq u_ref_seq_;
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
  /// lower bound vector, \f$\delta\ ^{qp}U_{min} \in \mathbb{R}^{mN}\f$
  Eigen::VectorX<real_t> lb_;
  /// upper bound vector, \f$\delta\ ^{qp}U_{max} \in \mathbb{R}^{mN}\f$
  Eigen::VectorX<real_t> ub_;
  /**
   * delta input vector, \f$\delta\ ^{qp}U = \left[\delta u_0^T, \delta
   * u_1^T, \ldots, \delta u_{N-1}^T\right]^T \in \mathbb{R}^{mN}\f$
   */
  Eigen::VectorX<real_t> dU_;

  qpOASES::QProblem qp_;
  bool solved_ = false;  ///< whether the QP problem is solved
};
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace robot_utils

#endif /* HAS_QPOASES */
#endif /* ROBOT_UTILS_CONTROLLER_OMMPC_HPP_ */
