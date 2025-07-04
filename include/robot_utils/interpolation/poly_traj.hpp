/**
 *******************************************************************************
 * @file poly_traj.hpp
 * @brief
 *
 * @section history
 *
 * @version 1.0.0
 * @date 2025-05-27
 * @author Caikunzhen
 * @details
 * 1. Complete the poly_traj.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_INTERPOLATION_POLY_TRAJ_HPP_
#define ROBOT_UTILS_INTERPOLATION_POLY_TRAJ_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>
#ifdef HAS_NLOPT
#include <yaml-cpp/yaml.h>

#include <nlopt.hpp>
#endif  // HAS_NLOPT
#include <type_traits>
#include <vector>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/typedef.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @brief Polynomial class
 *
 * This class represents a polynomial of the form:
 *
 * \f[
 * P\left(t\right) = CT\left(t\right)
 * \f]
 *
 * where \f$C \in \mathbb{R}^{d \times \left(n+1\right)}\f$ is the
 * polynomial coefficients, \f$d\f$ is the dimension of the polynomial,
 * and \f$n\f$ is the order of the polynomial, and \f$T\left(t\right)\f$ is
 * the polynomial basis vector:
 *
 * \f[
 * T\left(t\right) = \left[1, t, t^2, \ldots, t^n\right]^T
 * \f]
 *
 * @tparam T data type, only float and double are supported
 */
template <typename T>
class Poly
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "TdParams only supports float and double");

 public:
  using Ptr = std::shared_ptr<Poly<T>>;
  using ConstPtr = std::shared_ptr<const Poly<T>>;

  /**
   * @brief Polynomial constructor
   * @param coeff Polynomial coefficients, \f$C \in \mathbb{R}^{d \times
   * \left(n+1\right)}\f$
   */
  explicit Poly(const MatrixX<T>& coeff) : coeff_(coeff)
  {
    RU_ASSERT(coeff.cols() > 0,
              "Polynomial coefficients must have at least 1 column");
  }
  ~Poly(void) = default;

  /**
   * @brief Evaluate the polynomial at time \f$t\f$
   * @param[in] t Time, \f$t \in \mathbb{R}\f$
   * @param[in] n_diff Order of the derivative, \f$n_d \in \mathbb{N}\f$
   * @return Polynomial value, \f$P^\left(n_d\right)\left(t\right) \in
   * \mathbb{R}^d\f$
   */
  VectorX<T> operator()(const T& t, size_t n_diff = 0) const;

  /**
   * @brief Set the polynomial coefficients
   * @param[in] coeff Polynomial coefficients, \f$C \in \mathbb{R}^{d \times
   * \left(n+1\right)}\f$
   */
  void setCoeff(const MatrixX<T>& coeff) { coeff_ = coeff; }
  const MatrixX<T>& getCoeff(void) const { return coeff_; }

 private:
  /**
   * @brief polynomial coefficients, \f$C \in \mathbb{R}^{d \times
   * \left(n+1\right)}\f$
   */
  MatrixX<T> coeff_;
};

/**
 * @brief Polynomial trajectory class
 *
 * This class represents a polynomial trajectory. It has two versions, for time
 * scale and non-time scale.
 *
 * For time scale, the polynomial trajectory is defined as:
 *
 * \f[
 * P\left(t\right) = \begin{cases}
 * P_0\left(\left(t - t0\right) / \Delta t_0\right), & t < t_1 \\
 * P_1\left(\left(t - t1\right) / \Delta t_1\right), & t \in \left[t_1,
 * t_2\right) \\
 * \vdots \\
 * P_{M-1}\left(\left(t - t_{M-1}\right) / \Delta t_{M-1}\right), & t \ge
 * t_{M-1} \\
 * \end{cases}
 * \f]
 *
 * For non-time scale, the polynomial trajectory is defined as:
 *
 * \f[
 * P\left(t\right) = \begin{cases}
 * P_0\left(t - t_0\right), & t < t_1 \\
 * P_1\left(t - t_1\right), & t \in \left[t_1, t_2\right) \\
 * \vdots \\
 * P_{M-1}\left(t - t_{M-1}\right), & t \ge t_{M-1} \\
 * \end{cases}
 * \f]
 *
 * where \f$P_i\left(t\right)\f$ is the polynomial of the \f$i\f$-th
 * segment, \f$M\f$ is the number of segments, \f$t_0\f$ is the start
 * time, \f$t_M\f$ is the end time, \f$\Delta t_i = t_{i+1} - t_i\f$.
 *
 * @tparam T data type, only float and double are supported
 */
template <typename T>
class PolyTraj
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "PolyTraj only supports float and double");

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PolyTraj<T>>;
  using ConstPtr = std::shared_ptr<const PolyTraj<T>>;
  using Polys = std::vector<Poly<T>>;
  using PolyCoeffs = std::vector<MatrixX<T>>;

  enum CfgMask {
    kCfgMaskNone = 0,
    /// start clamp, \f$t\f$ will be set to \f$t_0\f$ if \f$t < t_0\f$
    kCfgMaskStartClamp = 1,
    /// end clamp, \f$t\f$ will be set to \f$t_M\f$ if \f$t > t_M\f$
    kCfgMaskEndClamp = 2,
  };

  /**
   * @brief Polynomial trajectory constructor
   * @param[in] dim Dimension of the polynomial, \f$d\f$
   * @param[in] cfg_mask Configuration mask, conbination of @ref CfgMask
   */
  explicit PolyTraj(size_t dim,
                    int cfg_mask = kCfgMaskStartClamp | kCfgMaskEndClamp);
  ~PolyTraj(void) = default;

  /**
   * @brief Evaluate the polynomial trajectory at time \f$t\f$
   * @param[in] t Time, \f$t \in \mathbb{R}\f$
   * @param[in] n_diff Order of the derivative, \f$n_d \in \mathbb{N}\f$
   * @return Polynomial trajectory value, \f$P^\left(n_d\right)\left(t\right)
   * \in \mathbb{R}^d\f$
   */
  VectorX<T> operator()(T t, size_t n_diff = 0) const;

  /**
   * @brief Set the polynomial trajectory
   * @param[in] t0 Start time, \f$t_0\f$
   * @param[in] polys Polynomial list, \f$P_i\left(t\right)\f$, \f$i = 0, 1,
   * \ldots, M-1\f$
   * @param[in] dts Time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$
   * @param[in] is_time_scale Whether the trajectory is time scale or not
   */
  void setTraj(T t0, const Polys& polys, const std::vector<T>& dts,
               bool is_time_scale = true)
  {
    RU_ASSERT(polys.size() == dts.size(),
              "polys.size() = %zu, dts.size() = %zu", polys.size(), dts.size());
    for (const auto& poly : polys) {
      RU_ASSERT(poly.getCoeff().rows() == dim_,
                "poly.getCoeff().rows() = %zu, dim_ = %zu",
                poly.getCoeff().rows(), dim_);
    }
    t0_ = t0;
    polys_ = polys;
    dts_ = dts;
    is_time_scale_ = is_time_scale;
    tf_ = t0_;
    for (const auto& dt : dts_) {
      tf_ += dt;
    }
  }
  /**
   * @brief Set the polynomial trajectory
   * @param[in] t0 Start time, \f$t_0\f$
   * @param[in] coeffs Polynomial coefficients, \f$\left[C_0, C_1, \ldots,
   * C_{M-1}\right]\f$, \f$C_i \in \mathbb{R}^{d \times \left(n+1\right)}\f$
   * @param[in] dts Time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$
   * @param[in] is_time_scale Whether the trajectory is time scale or not
   */
  void setTraj(T t0, const PolyCoeffs& coeffs, const std::vector<T>& dts,
               bool is_time_scale = true)
  {
    RU_ASSERT(coeffs.size() == dts.size(),
              "coeffs.size() = %zu, dts.size() = %zu", coeffs.size(),
              dts.size());
    t0_ = t0;
    polys_.clear();
    for (const auto& coeff : coeffs) {
      polys_.emplace_back(coeff);
    }
    dts_ = dts;
    is_time_scale_ = is_time_scale;
    tf_ = t0_;
    for (const auto& dt : dts_) {
      tf_ += dt;
    }
  }

  size_t getDim(void) const { return dim_; }
  const Polys& getPolys(void) const { return polys_; }
  const std::vector<T>& getDts(void) const { return dts_; }
  T getStartTime(void) { return t0_; }
  T getEndTime(void) const { return tf_; }

 private:
  void getPolyIdxAndDt(const T& t, size_t& idx, T& dt) const;

  const size_t dim_ = 0;  ///< dimension of the polynomial, \f$d\f$
  int cfg_mask_ = kCfgMaskStartClamp | kCfgMaskEndClamp;
  Polys polys_;  ///< polynomial list
  /**
   * @brief time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$
   */
  std::vector<T> dts_;
  T t0_ = 0;  ///< start time, \f$t_0\f$
  T tf_ = 0;  ///< end time, \f$t_M\f$
  bool is_time_scale_ = true;
};

/**
 * @brief Polynomial trajectory optimization class
 *
 * This class represents a polynomial trajectory optimization problem. It
 * solves the following problem:
 *
 * \f[
 * \underset{C_{0:M-1}}{\min} J = \int_{t_0}^{t_M} \left
 * \|P^\left(r\right)\left(t\right)\right \|^2_2 dt
 * \f]
 *
 * s.t.
 *
 * \f[
 * P^{\left(k\right)}\left(t_0\right) = x_0^{\left(k\right)}, k = 0, 1, \ldots,
 * r -
 * 1
 * \f]
 *
 * \f[
 * P^{\left(k\right)}\left(t_M\right) = x_M^{\left(k\right)}, k = 0, 1, \ldots,
 * r -
 * 1
 * \f]
 *
 * \f[
 * P\left(t_i\right) = x_i, i = 1, 2, \ldots, M - 1
 * \f]
 *
 * \f[
 * P^{\left(k\right)}\left(t_i\right) = P^{\left(k\right)}\left(t_{i+1}\right),
 * k = 0, 1, \ldots, r - 1 \text{ and } i = 1, 2, \ldots, M - 2
 * \f]
 *
 * where \f$P\left(t\right)\f$ is the polynomial, \f$C_{0:M-1} = \left[C_0,
 * C_1, \ldots, C_{M-1}\right]\f$ is the polynomial coefficients, details can be
 * found in @ref PolyTraj, \f$x_i\f$ is the \f$i\f$-th waypoint, \f$M\f$ is the
 * number of trajetory segments.
 *
 * @tparam T data type, only float and double are supported
 * @tparam Dim dimension of the polynomial, \f$d\f$, only 1, 2 and 3 are
 * supported
 * @tparam EnergyOrder order of the energy function, \f$r\f$, only
 * 2(acceleration), 3(jerk) and 4(snap) are supported
 */
template <typename T, size_t Dim, size_t EnergyOrder>
class PolyTrajOpt
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "PolyTrajOpt only supports float and double");
  static_assert(EnergyOrder == 2 || EnergyOrder == 3 || EnergyOrder == 4,
                "EnergyOrder only supports 2, 3 or 4");

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Ptr = std::shared_ptr<PolyTrajOpt<T, Dim, EnergyOrder>>;
  using ConstPtr = std::shared_ptr<const PolyTrajOpt<T, Dim, EnergyOrder>>;

  using State = Eigen::Matrix<T, Dim, EnergyOrder>;
  using Waypoint = Eigen::Vector<T, Dim>;
  using Waypoints = std::vector<Waypoint>;

  PolyTrajOpt(void) = default;
  ~PolyTrajOpt(void) = default;

  /**
   * @brief Optimize the polynomial trajectory
   * @param[in] x0 Start state, \f$x_0 \in \mathbb{R}^d\f$
   * @param[in] xf End state, \f$x_M \in \mathbb{R}^d\f$
   * @param[in] xi Waypoints, \f$\left[x_1, x_2, \ldots, x_{M-1}\right], x_i \in
   * \mathbb{R}^d\f$
   * @param[in] dts Time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$
   * @param[in] t0 Start time, \f$t_0\f$
   * @param[out] traj Polynomial trajectory, \f$P\left(t\right)\f$
   * @param[out] J Cost function value, \f$J\f$, optional
   * @param[out] partial_grad_by_dts Partial gradient of the cost function with
   * respect to the time intervals, \f$\frac{\partial J}{\partial \tau} \in
   * \mathbb{R}^M\f$ where \f$\tau = \left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]^T\f$, can be used to optimize the time intervals,
   * optional
   * @note The partial gradient of the cost function with respect to the
   * time intervals \f$\frac{\partial J}{\partial \tau}\f$ is often very
   * large, so it is recommended to use the normalized gradient.
   */
  void optimize(const State& x0, const State& xf, const Waypoints& xi, T t0,
                const std::vector<T>& dts, PolyTraj<T>& traj, T* J = nullptr,
                VectorX<T>* partial_grad_by_dts = nullptr);

  /**
   * @brief Allocate time intervals for the polynomial trajectory
   * @param[in] x0 Start state, \f$x_0 \in \mathbb{R}^d\f$
   * @param[in] xf End state, \f$x_M \in \mathbb{R}^d\f$
   * @param[in] xi Waypoints, \f$\left[x_1, x_2, \ldots, x_{M-1}\right], x_i \in
   * \mathbb{R}^d\f$
   * @param[in] max_vel Maximum velocity
   * @param[in] max_acc Maximum acceleration
   * @param[out] dts Time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$
   * @note `max_vel` and `max_acc` are used to allocate time intervals for the
   * trajectory but not used in the optimization process, which means that
   * the trajectory may not satisfy the velocity and acceleration
   * constraints.
   */
  void simpleTimeAllocate(const State& x0, const State& xf, const Waypoints& xi,
                          T max_vel, T max_acc, std::vector<T>& dts) const;

 private:
  Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder> getQDur2R_1(void) const;

  Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder> getAiInv(size_t i) const;

  Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder> getHi(size_t i) const;

  Eigen::Matrix<T, 2 * EnergyOrder, 2 * EnergyOrder> getHiPartialGradByDti(
      size_t i) const;

  Eigen::SparseMatrix<T> getCT(void) const;

  MatrixX<T> getH(void) const;

  MatrixX<T> getRPartalGradByDti(size_t i) const;

  VectorX<T> getEnergyPartialGradByDt(const MatrixX<T>& R_PP_inv_R_PF,
                                      const MatrixX<T>& Df) const;

  static constexpr size_t _R = EnergyOrder;

  size_t M_ = 0;
  MatrixX<T> C_T_;
  VectorX<T> dts_;
  VectorX<T> dt2s_;
  VectorX<T> dt3s_;
  VectorX<T> dt4s_;
  VectorX<T> dt5s_;
  VectorX<T> dt6s_;
  VectorX<T> dt7s_;
  VectorX<T> dt8s_;
};

#ifdef HAS_NLOPT

/**
 * @brief Parameters for polynomial trajectory optimization with time
 * optimization
 * @note This struct is only available if NLOPT is installed and configured.
 */
struct PolyTrajOptWithTimeOptParams {
  /// relative tolerance for variable, 0 means no tolerance
  double xtol_rel = 1e-2;
  /// absolute tolerance for variable, 0 means no tolerance
  double xtol_abs = 0;
  /// relative tolerance for function value, 0 means no tolerance
  double ftol_rel = 0;
  /// absolute tolerance for function value, 0 means no tolerance
  double ftol_abs = 0;
  double maxeval = 0;  ///< maximum number of evaluations, 0 means no limit
  double maxtime = 0;  ///< maximum time, 0 means no limit

  /**
   * @brief weight for the time optimization, this is used to ensure that the
   * total time remains the same as the original trajectory
   */
  double weight_t = 1e5;

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * xtol_rel: 1e-2 # optional
   * xtol_abs: 0 # optional
   * ftol_rel: 0 # optional
   * ftol_abs: 0 # optional
   * maxeval: 0 # optional
   * maxtime: 0 # optional
   *
   * weight_t: 1e5
   * ```
   *
   * @param node YAML node containing the parameters
   * @param params Parameters for the optimization
   */
  static void LoadParamsFromYamlNode(const YAML::Node& node,
                                     PolyTrajOptWithTimeOptParams& params)
  {
    if (node["xtol_rel"]) {
      params.xtol_rel = node["xtol_rel"].as<double>();
    }
    if (node["xtol_abs"]) {
      params.xtol_abs = node["xtol_abs"].as<double>();
    }
    if (node["ftol_rel"]) {
      params.ftol_rel = node["ftol_rel"].as<double>();
    }
    if (node["ftol_abs"]) {
      params.ftol_abs = node["ftol_abs"].as<double>();
    }
    if (node["maxeval"]) {
      params.maxeval = node["maxeval"].as<double>();
    }
    if (node["maxtime"]) {
      params.maxtime = node["maxtime"].as<double>();
    }
    params.weight_t = node["weight_t"].as<double>();
  }

  /**
   * @brief Load parameters from YAML node
   *
   * This function loads the parameters from a YAML node. The YAML node should
   * contain the following parameters:
   *
   * ```yaml
   * xtol_rel: 1e-2 # optional
   * xtol_abs: 0 # optional
   * ftol_rel: 0 # optional
   * ftol_abs: 0 # optional
   * maxeval: 0 # optional
   * maxtime: 0 # optional
   *
   * weight_t: 1e5
   * ```
   *
   * @param node YAML node containing the parameters
   * @return Parameters for the optimization
   */
  static PolyTrajOptWithTimeOptParams LoadParamsFromYamlNode(
      const YAML::Node& node)
  {
    PolyTrajOptWithTimeOptParams params;
    LoadParamsFromYamlNode(node, params);
    return params;
  }
};

/**
 * @brief Polynomial trajectory optimization with time optimization class
 *
 * This class represents a polynomial trajectory optimization problem with time
 * optimization. It solves the following problem:
 *
 * \f[
 * \underset{C_{0:M-1}, t_{1:M-1}}{\min} J = \int_{t_0}^{t_M} \left
 * \|P^\left(r\right)\left(t\right)\right \|^2_2 dt
 * \f]
 *
 * The constraints are the same as @ref PolyTrajOpt, but the time intervals
 * \f$\left[\Delta t_0, \Delta t_1, \ldots, \Delta t_{M-1}\right]\f$ are
 * optimized.
 *
 * @tparam Dim dimension of the polynomial, \f$d\f$, only 1, 2 and 3 are
 * supported
 * @tparam EnergyOrder order of the energy function, \f$r\f$, only
 * 2(acceleration), 3(jerk) and 4(snap) are supported
 * @note This class requires the NLOPT library to be installed and
 * configured. If NLOPT is not available, this class will not be compiled.
 */
template <size_t Dim, size_t EnergyOrder>
class PolyTrajOptWithTimeOpt : public PolyTrajOpt<double, Dim, EnergyOrder>
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Params = PolyTrajOptWithTimeOptParams;
  using State = typename PolyTrajOpt<double, Dim, EnergyOrder>::State;
  using Waypoint = typename PolyTrajOpt<double, Dim, EnergyOrder>::Waypoint;
  using Waypoints = typename PolyTrajOpt<double, Dim, EnergyOrder>::Waypoints;

  using Ptr = std::shared_ptr<PolyTrajOptWithTimeOpt<Dim, EnergyOrder>>;
  using ConstPtr =
      std::shared_ptr<const PolyTrajOptWithTimeOpt<Dim, EnergyOrder>>;

  PolyTrajOptWithTimeOpt(const Params& params) : params_(params) {}
  ~PolyTrajOptWithTimeOpt(void);

  /**
   * @brief Optimize the polynomial trajectory with time optimization
   * @param[in] x0 Start state, \f$x_0 \in \mathbb{R}^d\f$
   * @param[in] xf End state, \f$x_M \in \mathbb{R}^d\f$
   * @param[in] xi Waypoints, \f$\left[x_1, x_2, \ldots, x_{M-1}\right], x_i \in
   * \mathbb{R}^d\f$
   * @param[in] t0 Start time, \f$t_0\f$
   * @param[in,out] dts Time intervals, \f$\left[\Delta t_0, \Delta t_1, \ldots,
   * \Delta t_{M-1}\right]\f$, input is the initial guess for the time
   * intervals, output is the optimized time intervals
   * @param[out] traj Polynomial trajectory, \f$P\left(t\right)\f$
   * @param[out] J Cost function value, \f$J\f$, optional
   */
  void optimizeWithTimeOpt(const State& x0, const State& xf,
                           const Waypoints& xi, double t0,
                           std::vector<double>& dts, PolyTraj<double>& traj,
                           double* J = nullptr);

  void setParams(const Params& params) { params_ = params; }
  const Params& getParams(void) const { return params_; }

 private:
  static double objectiveFunc(const std::vector<double>& x,
                              std::vector<double>& grad, void* data);

  /// smooth factor for the total time remain unchanged
  static constexpr double kSmoothMu = 1e-2;

  Params params_;
  nlopt::opt opt_;

  // The following members are used to store the optimization problem

  const State* x0_ = nullptr;
  const State* xf_ = nullptr;
  const Waypoints* xi_ = nullptr;
  double t0_ = 0;
  double t_total_ = 0;
  PolyTraj<double>* traj_ = nullptr;
};
#endif  // HAS_NLOPT

extern template class Poly<float>;
using Polyf = Poly<float>;
extern template class Poly<double>;
using Polyd = Poly<double>;
extern template class PolyTraj<float>;
using PolyTrajf = PolyTraj<float>;
extern template class PolyTraj<double>;
using PolyTrajd = PolyTraj<double>;

extern template class PolyTrajOpt<float, 1, 2>;
using PolyTrajMinAcc1f = PolyTrajOpt<float, 1, 2>;
extern template class PolyTrajOpt<double, 1, 2>;
using PolyTrajMinAcc1d = PolyTrajOpt<double, 1, 2>;
extern template class PolyTrajOpt<float, 2, 2>;
using PolyTrajMinAcc2f = PolyTrajOpt<float, 2, 2>;
extern template class PolyTrajOpt<double, 2, 2>;
using PolyTrajMinAcc2d = PolyTrajOpt<double, 2, 2>;
extern template class PolyTrajOpt<float, 3, 2>;
using PolyTrajMinAcc3f = PolyTrajOpt<float, 3, 2>;
extern template class PolyTrajOpt<double, 3, 2>;
using PolyTrajMinAcc3d = PolyTrajOpt<double, 3, 2>;
extern template class PolyTrajOpt<float, 1, 3>;
using PolyTrajMinJerk1f = PolyTrajOpt<float, 1, 3>;
extern template class PolyTrajOpt<double, 1, 3>;
using PolyTrajMinJerk1d = PolyTrajOpt<double, 1, 3>;
extern template class PolyTrajOpt<float, 2, 3>;
using PolyTrajMinJerk2f = PolyTrajOpt<float, 2, 3>;
extern template class PolyTrajOpt<double, 2, 3>;
using PolyTrajMinJerk2d = PolyTrajOpt<double, 2, 3>;
extern template class PolyTrajOpt<float, 3, 3>;
using PolyTrajMinJerk3f = PolyTrajOpt<float, 3, 3>;
extern template class PolyTrajOpt<double, 3, 3>;
using PolyTrajMinJerk3d = PolyTrajOpt<double, 3, 3>;
extern template class PolyTrajOpt<float, 1, 4>;
using PolyTrajMinSnap1f = PolyTrajOpt<float, 1, 4>;
extern template class PolyTrajOpt<double, 1, 4>;
using PolyTrajMinSnap1d = PolyTrajOpt<double, 1, 4>;
extern template class PolyTrajOpt<float, 2, 4>;
using PolyTrajMinSnap2f = PolyTrajOpt<float, 2, 4>;
extern template class PolyTrajOpt<double, 2, 4>;
using PolyTrajMinSnap2d = PolyTrajOpt<double, 2, 4>;
extern template class PolyTrajOpt<float, 3, 4>;
using PolyTrajMinSnap3f = PolyTrajOpt<float, 3, 4>;
extern template class PolyTrajOpt<double, 3, 4>;
using PolyTrajMinSnap3d = PolyTrajOpt<double, 3, 4>;

#ifdef HAS_NLOPT
extern template class PolyTrajOptWithTimeOpt<1, 2>;
using PolyTrajMinAccWithTimeOpt1 = PolyTrajOptWithTimeOpt<1, 2>;
extern template class PolyTrajOptWithTimeOpt<2, 2>;
using PolyTrajMinAccWithTimeOpt2 = PolyTrajOptWithTimeOpt<2, 2>;
extern template class PolyTrajOptWithTimeOpt<3, 2>;
using PolyTrajMinAccWithTimeOpt3 = PolyTrajOptWithTimeOpt<3, 2>;
extern template class PolyTrajOptWithTimeOpt<1, 3>;
using PolyTrajMinJerkWithTimeOpt1 = PolyTrajOptWithTimeOpt<1, 3>;
extern template class PolyTrajOptWithTimeOpt<2, 3>;
using PolyTrajMinJerkWithTimeOpt2 = PolyTrajOptWithTimeOpt<2, 3>;
extern template class PolyTrajOptWithTimeOpt<3, 3>;
using PolyTrajMinJerkWithTimeOpt3 = PolyTrajOptWithTimeOpt<3, 3>;
extern template class PolyTrajOptWithTimeOpt<1, 4>;
using PolyTrajMinSnapWithTimeOpt1 = PolyTrajOptWithTimeOpt<1, 4>;
extern template class PolyTrajOptWithTimeOpt<2, 4>;
using PolyTrajMinSnapWithTimeOpt2 = PolyTrajOptWithTimeOpt<2, 4>;
extern template class PolyTrajOptWithTimeOpt<3, 4>;
using PolyTrajMinSnapWithTimeOpt3 = PolyTrajOptWithTimeOpt<3, 4>;
#endif  // HAS_NLOPT
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief Generate a polynomial trajectory segment with uniform acceleration
 *
 * Problem formulation:
 *
 * \f[
 * \underset{P}{\min} J = t_f - t_0
 * \f]
 *
 * s.t.
 *
 * \f[
 * P\left(t_0\right) = x_0
 * \f]
 *
 * \f[
 * P^{'}\left(t_0\right) = v_0
 * \f]
 *
 * \f[
 * P\left(t_f\right) = x_f
 * \f]
 *
 * \f[
 * P^{'}\left(t_f\right) = 0
 * \f]
 *
 * \f[
 * P^{'}\left(t\right) \in \left[-v_m, v_m\right]
 * \f]
 *
 * \f[
 * P^{''}\left(t\right) \in \left[-a_m, a_m\right]
 * \f]
 *
 * @tparam T data type, only float and double are supported
 * @param[in] max_vel Maximum velocity, \f$v_m\f$
 * @param[in] max_acc Maximum acceleration, \f$a_m\f$
 * @param[in] t0 Start time, \f$t_0\f$
 * @param[in] x0 Start position, \f$[x_0, v_0]^T\f$, where \f$x_0\f$ is the
 * start position and \f$v_0\f$ is the start velocity
 * @param[in] xf End position, \f$x_f\f$, the end velocity is 0
 * @param[out] traj Output polynomial trajectory segment, \f$P\left(t\right)\f$
 */
template <typename T>
void PolyTrajSegUniAcc(T max_vel, T max_acc, T t0, const Eigen::Vector2<T>& x0,
                       T xf, PolyTraj<T>& traj);
}  // namespace robot_utils

#endif /* ROBOT_UTILS_INTERPOLATION_POLY_TRAJ_HPP_ */
