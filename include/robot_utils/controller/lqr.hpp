/**
 *******************************************************************************
 * @file lqr.hpp
 * @brief Linear Quadratic Regulator (LQR) controller
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-10
 * @author Caikunzhen
 * @details
 * 1. Complete the lqr.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_CONTROLLER_LQR_HPP_
#define ROBOT_UTILS_CONTROLLER_LQR_HPP_

/* Includes ------------------------------------------------------------------*/
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <vector>

#include "robot_utils/core/assert.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Parameters of LQR controller
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
struct LqrParams {
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "LqrParam only supports float and double");

  using DiagMatX = Eigen::DiagonalMatrix<T, Eigen::Dynamic>;

  size_t n = 0;                  ///< number of states, \f$n\f$
  size_t m = 0;                  ///< number of inputs, \f$m\f$
  size_t max_iter = 100;         ///< maximum number of iterations
  T tol = static_cast<T>(1e-6);  ///< tolerance for convergence

  Eigen::MatrixX<T> A;  ///< system matrix, \f$A \in \mathbb{R}^{n \times n}\f$
  Eigen::MatrixX<T> B;  ///< input matrix, \f$B \in \mathbb{R}^{n \times m}\f$
  DiagMatX Q;  ///< state cost matrix, \f$Q \in \mathbb{R}^{n \times n}\f$
  DiagMatX R;  ///< input cost matrix, \f$R \in \mathbb{R}^{m \times m}\f$

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * n: <number of states>
   * m: <number of inputs>
   * max_iter: <maximum number of iterations>
   * tol: <tolerance for convergence>
   * A: [<A matrix values>] # optional, size (1, n * n), row-major order
   * B: [<B matrix values>] # optional, size (1, n * m), row-major order
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @param[out] params: LQR parameters
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node, LqrParams& params)
  {
    params.n = node["n"].as<size_t>();
    params.m = node["m"].as<size_t>();
    params.max_iter = node["max_iter"].as<size_t>();
    params.tol = node["tol"].as<T>();

    if (node["A"]) {
      params.A.resize(params.n, params.n);
      std::vector<T> A_flat = node["A"].as<std::vector<T>>();
      RU_ASSERT(
          A_flat.size() == params.n * params.n,
          "A matrix size is not correct, expected size: %d, actual size: %d",
          params.n * params.n, A_flat.size());
      for (size_t i = 0; i < params.n; ++i) {
        for (size_t j = 0; j < params.n; ++j) {
          params.A(i, j) = A_flat[i * params.n + j];
        }
      }
    }

    if (node["B"]) {
      params.B.resize(params.n, params.m);
      std::vector<T> B_flat = node["B"].as<std::vector<T>>();
      RU_ASSERT(
          B_flat.size() == params.n * params.m,
          "B matrix size is not correct, expected size: %d, actual size: %d",
          params.n * params.m, B_flat.size());
      for (size_t i = 0; i < params.n; ++i) {
        for (size_t j = 0; j < params.m; ++j) {
          params.B(i, j) = B_flat[i * params.m + j];
        }
      }
    }

    std::vector<T> Q_diag = node["Q_diag"].as<std::vector<T>>();
    params.Q.resize(params.n);
    RU_ASSERT(
        Q_diag.size() == params.n,
        "Q matrix size is not correct, expected size: %d, actual size: %d",
        params.n, Q_diag.size());
    for (size_t i = 0; i < params.n; ++i) {
      params.Q.diagonal()[i] = Q_diag[i];
    }

    std::vector<T> R_diag = node["R_diag"].as<std::vector<T>>();
    params.R.resize(params.m);
    RU_ASSERT(
        R_diag.size() == params.m,
        "R matrix size is not correct, expected size: %d, actual size: %d",
        params.m, R_diag.size());
    for (size_t i = 0; i < params.m; ++i) {
      params.R.diagonal()[i] = R_diag[i];
    }
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * n: <number of states>
   * m: <number of inputs>
   * max_iter: <maximum number of iterations>
   * tol: <tolerance for convergence>
   * A: [<A matrix values>] # optional, size (1, n * n), row-major order
   * B: [<B matrix values>] # optional, size (1, n * m), row-major order
   * Q_diag: [<Q matrix diagonal values>] # size (1, n)
   * R_diag: [<R matrix diagonal values>] # size (1, m)
   * ```
   *
   * @param[in] node: YAML node containing the parameters
   * @return LQR parameters
   */
  static LqrParams LoadParamsFromYamlNode(const YAML::Node& node)
  {
    LqrParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief LQR controller
 *
 * System dynamics:
 *
 * \f[
 * x_{k+1} = Ax_k + Bu_k
 * \f]
 *
 * Cost function:
 *
 * \f[
 * J = \sum_{k=0}^{\infty} \left(\left(x^d - x_k\right)^T Q \left(x^d -
 * x_k\right) + u_k^T R u_k\right)
 * \f]
 *
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class Lqr
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "Lqr only supports float and double");

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = LqrParams<T>;
  using Ptr = std::shared_ptr<Lqr>;
  using ConstPtr = std::shared_ptr<const Lqr>;
  /// state vector, \f$x \in \mathbb{R}^n\f$
  using StateVec = Eigen::VectorX<T>;
  /// input vector, \f$u \in \mathbb{R}^m\f$
  using InputVec = Eigen::VectorX<T>;

  struct Data {
    /// feedback gain matrix, \f$K \in \mathbb{R}^{m \times n}\f$
    Eigen::MatrixX<T> K;
    /// solution to the Riccati equation, \f$P \in \mathbb{R}^{n \times n}\f$
    Eigen::MatrixX<T> P;
    T res = std::numeric_limits<T>::max();  ///< residual of the cost function
    bool is_converged = false;
  };

  explicit Lqr(const Params& params);
  virtual ~Lqr(void) = default;

  /**
   * @brief Solve the LQR problem
   * @return true if converged, false if not
   */
  bool solve(void);

  /**
   * @brief Calculate the LQR control input
   *
   * This function calculates the LQR control input using the reference state
   * vector and the feedback state vector. The LQR control input is calculated
   * as:
   *
   * \f[
   * u_k = K \cdot \left(x^d - x_k\right)
   * \f]
   *
   * @param[in] ref: Reference state vector, \f$x^d \in \mathbb{R}^n\f$
   * @param[in] fdb: Feedback state vector, \f$x_k \in \mathbb{R}^n\f$
   * @param[out] u: Control input vector, \f$u_k \in \mathbb{R}^m\f$
   * @note Call @ref solve before calling this function and it must be
   * converged. Otherwise, the result is undefined.
   */
  void calc(const StateVec& ref, const StateVec& fdb, InputVec& u) const;

  /**
   * @brief Calculate the LQR control input
   *
   * This function calculates the LQR control input using the reference state
   * vector and the feedback state vector. The LQR control input is calculated
   * as:
   *
   * \f[
   * u_k = K \cdot \left(x^d - x_k\right)
   * \f]
   *
   * @param[in] ref: Reference state vector, \f$x^d \in \mathbb{R}^n\f$
   * @param[in] fdb: Feedback state vector, \f$x_k \in \mathbb{R}^n\f$
   * @return Control input vector, \f$u_k \in \mathbb{R}^m\f$
   * @note Call @ref solve before calling this function and it must be
   * converged. Otherwise, the result is undefined.
   */
  InputVec calc(const StateVec& ref, const StateVec& fdb) const
  {
    InputVec u(params_.m);
    calc(ref, fdb, u);
    return u;
  }

  /**
   * @brief Set the parameters of the LQR controller
   * @param params: LQR parameters(`n` and `m` will be ignored)
   */
  void setParams(const Params& params);
  const Params& getParams(void) const { return params_; }

  const Data& getData(void) const { return data_; }

 private:
  Params params_;
  Data data_;
};

extern template class Lqr<float>;
using Lqrf = Lqr<float>;
extern template class Lqr<double>;
using Lqrd = Lqr<double>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_CONTROLLER_LQR_HPP_ */
