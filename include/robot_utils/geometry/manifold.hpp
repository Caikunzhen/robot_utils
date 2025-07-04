/**
 *******************************************************************************
 * @file manifold.hpp
 * @brief This file provides the manifold class for robot_utils, it is based on
 * the manifold section of paper:
 *
 * G. Lu, W. Xu and F. Zhang, "On-Manifold Model Predictive Control for
 * Trajectory Tracking on Robotic Systems," in IEEE Transactions on Industrial
 * Electronics, vol. 70, no. 9, pp. 9192-9202, Sept. 2023,
 * doi: 10.1109/TIE.2022.3212397.
 *
 * @section history
 *
 * @version V1.0.0
 * @date 2025-05-07
 * @author Caikunzhen
 * @details
 * 1. Complete the manifold.hpp
 *******************************************************************************
 *  Copyright (c) 2025 Caikunzhen, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef ROBOT_UTILS_GEOMETRY_MANIFOLD_HPP_
#define ROBOT_UTILS_GEOMETRY_MANIFOLD_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <memory>
#include <type_traits>
#include <variant>
#include <vector>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/periodic_data.hpp"
#include "robot_utils/core/typedef.hpp"
#include "robot_utils/geometry/core.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/

/**
 * @enum ManifoldType
 * @brief Manifold type
 */
enum class ManifoldType {
  kEuclideanSpaceX,  ///< Euclidean space, \f$\mathcal{M} = \mathbb{R}^n\f$
  /// Special orthogonal group, \f$\mathcal{M} = SO\left(2\right)\f$
  kSpecialOrthogonalGroup2,
  /// Special orthogonal group, \f$\mathcal{M} = SO\left(3\right)\f$
  kSpecialOrthogonalGroup3,
  /**
   * @brief 2-D Surface, \f$\mathcal{M} = \mathcal{S}, \mathcal{S} \doteq
   * \left\{\mathrm{x} \in \mathbb{R}^3 | z = F\left(x, y\right)\right\}\f$
   */
  kSurface2D,
  /// The cartesian product of arbitrary number of primitive manifolds
  kCompoundManifold
};

/**
 * @brief Base class for manifold
 *
 * Parameterized by a set of local coordinate charts \f$\left(U_\mathrm{x},
 * \varphi_\mathrm{x}\right)\f$, where \f$\varphi_{\mathrm{x}} : \mathcal{M}
 * \mapsto \mathbb{R}^n\f$ is a map project the neighborhood \f$U_\mathrm{x}
 * \subset \mathcal{M}\f$ around a point \f$\mathrm{x}\f$ to the linear
 * homeomorphic space \f$\mathbb{R}^n\f$. The region of the neighborhood
 * \f$U_\mathrm{x}\f$ is related to the choise of \f$\varphi_\mathrm{x}\f$ and
 * homeomorphic to an \f$n\f$-ball
 * \f$\mathbf{B}_\epsilon^n\left(\mathrm{x}\right) \in \mathbb{R}^n\f$ where
 * \f$\left\|\mathbf{B}_\epsilon^n\left(\mathrm{x}\right)\right\| <
 * \epsilon\f$ and \f$\epsilon > 0\f$.
 *
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class ManifoldBase
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ManifoldBase only supports float and double");

 public:
  using Scalar = T;
  using Type = ManifoldType;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<ManifoldBase<T>>;
  using ConstPtr = std::shared_ptr<const ManifoldBase<T>>;

  explicit ManifoldBase(Type type) : type_(type) {}
  virtual ~ManifoldBase(void) = default;

  /**
   * @brief Get the dimension of the manifold
   * @return The dimension of the manifold, \f$n\f$
   */
  virtual size_t dim(void) const = 0;

  Type type(void) const { return type_; }

  /**
   * @brief Check if the manifold is belongs to the same manifold
   * @param[in] other: The other manifold
   * @return true if the manifold is the same, false otherwise
   */
  virtual bool isSame(const ManifoldBase<T>& other) const
  {
    return type_ == other.type_ && dim() == other.dim();
  }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const = 0;

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const = 0;

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * \f$\boxplus : \mathcal{M} \times \mathbb{R}^n \mapsto \mathcal{M}\f$,
   * \f$\mathrm{x} \boxplus \delta =
   * \varphi_{\mathrm{x}}^{-1}\left(\delta\right), \forall \delta \in
   * \mathbf{B}_\epsilon^n\left(\mathrm{x}\right)\f$
   *
   * \f$\boxplus\f$ applies a small change or perturbation \f$\delta\f$ in the
   * homoeomorphic space at a point to yield a new point on the manifold
   * \f$\mathcal{M}\f$.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @param[out] y: The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$(must belong to the same manifold as this, details see
   * @ref isSame)
   */
  void add(const HomeSpace& delta, ManifoldBase<T>& y) const
  {
    invProj(delta, y);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * \f$\boxminus : \mathcal{M} \times \mathcal{M} \mapsto \mathbb{R}^n\f$,
   * \f$\mathrm{y} \boxminus \mathrm{x} =
   * \varphi_{\mathrm{x}}\left(\mathrm{y}\right), \forall \mathrm{y} \in
   * U_\mathrm{x}\f$
   *
   * \f$\boxminus\f$ represents the difference between two points on the
   * manifold in the homoeomorphic space.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x}\f$(must belong to
   * the same manifold as this, details see @ref isSame)
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const ManifoldBase<T>& x) const { return x.proj(*this); }

 protected:
  const Type type_;
};

/**
 * @brief Class for Euclidean space, \f$\mathcal{M} = \mathbb{R}^n\f$
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class EuclideanSpaceX : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "EuclideanSpaceX only supports float and double");

 public:
  using Scalar = T;
  using Data = VectorX<T>;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<EuclideanSpaceX<T>>;
  using ConstPtr = std::shared_ptr<const EuclideanSpaceX<T>>;

  /**
   * @brief Constructor of the Euclidean space
   * @param[in] dim: The dimension of the Euclidean space, \f$n\f$
   */
  explicit EuclideanSpaceX(size_t dim)
      : ManifoldBase<T>(ManifoldType::kEuclideanSpaceX), dim_(dim)
  {
    RU_ASSERT(dim > 0, "EuclideanSpaceX dimension must be greater than 0");

    m_.resize(dim);
    m_.setZero();
  }
  /**
   * @brief Constructor of the Euclidean space
   * @param[in] m: The point in the Euclidean space, \f$\mathrm{m} \in
   * \mathbb{R}^n\f$
   */
  explicit EuclideanSpaceX(const Data& m)
      : ManifoldBase<T>(ManifoldType::kEuclideanSpaceX), m_(m), dim_(m.size())
  {
    RU_ASSERT(m.size() > 0, "EuclideanSpaceX dimension must be greater than 0");
  }
  virtual ~EuclideanSpaceX(void) = default;

  /**
   * @brief Get the dimension of the Euclidean space
   * @return The dimension of the Euclidean space, \f$n\f$
   */
  virtual size_t dim(void) const override { return dim_; }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    const EuclideanSpaceX<T>& y_t = (const EuclideanSpaceX<T>&)y;

    return y_t.m_ - m_;
  }

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    EuclideanSpaceX<T>& y_t = (EuclideanSpaceX<T>&)y;

    y_t.m_ = m_ + phi;
  }

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * Details see @ref ManifoldBase::add.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @return The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$
   */
  EuclideanSpaceX operator+(const HomeSpace& delta) const
  {
    RU_ASSERT(delta.size() == dim_,
              "EuclideanSpaceX dimension mismatch, expected %zu, got %zu", dim_,
              delta.size());

    return EuclideanSpaceX(m_ + delta);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * Details see @ref ManifoldBase::operator-.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x} \in \mathbb{R}^n\f$
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const EuclideanSpaceX& x) const
  {
    RU_ASSERT(x.dim() == dim_,
              "EuclideanSpaceX dimension mismatch, expected %zu, got %zu", dim_,
              x.dim());

    return m_ - x.m_;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  /**
   * @brief Get the identity of the Euclidean space
   * @return The identity of the Euclidean space, \f$\mathrm{x} =
   * \mathbf{0}_n\f$
   */
  EuclideanSpaceX<T> getIdentity(void) const
  {
    return EuclideanSpaceX<T>(dim_);
  }

 private:
  Data m_;
  size_t dim_ = 0;
};

/**
 * @brief Class for special orthogonal group, \f$\mathcal{M} = SO(2)\f$
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class SpecialOrthogonalGroup2 : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "SpecialOrthogonalGroup2 only supports float and double");

 public:
  using Scalar = T;
  using Data = Matrix2<T>;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<SpecialOrthogonalGroup2<T>>;
  using ConstPtr = std::shared_ptr<const SpecialOrthogonalGroup2<T>>;

  /**
   * @brief Constructor of the special orthogonal group
   *
   * This constructor initializes the special orthogonal group to the identity
   * matrix.
   */
  SpecialOrthogonalGroup2(void)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup2)
  {
    m_.setIdentity();
  }
  /**
   * @brief Constructor of the special orthogonal group
   * @param[in] m: The point in the special orthogonal group, \f$\mathrm{m} \in
   * SO\left(2\right)\f$
   */
  explicit SpecialOrthogonalGroup2(const Data& m)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup2)
  {
    m_ = m;
  }
  virtual ~SpecialOrthogonalGroup2(void) = default;

  /**
   * @brief Get the dimension of the special orthogonal group
   * @return The dimension of the special orthogonal group
   */
  virtual size_t dim(void) const override { return 1; }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    const SpecialOrthogonalGroup2<T>& y_t =
        (const SpecialOrthogonalGroup2<T>&)y;

    Matrix2<T> m_delta = m_.transpose() * y_t.m_;
    HomeSpace delta(1);
    delta(0) = std::atan2(m_delta(1, 0), m_delta(0, 0));
    return delta;
  }

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    SpecialOrthogonalGroup2<T>& y_t = (SpecialOrthogonalGroup2<T>&)y;

    Matrix2<T> m_delta;
    m_delta << std::cos(phi(0)), -std::sin(phi(0)), std::sin(phi(0)),
        std::cos(phi(0));
    y_t.m_ = m_ * m_delta;
  }

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * Details see @ref ManifoldBase::add.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @return The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$
   */
  SpecialOrthogonalGroup2 operator+(const HomeSpace& delta) const
  {
    RU_ASSERT(
        delta.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        delta.size());

    Matrix2<T> m_delta;
    m_delta << std::cos(delta(0)), -std::sin(delta(0)), std::sin(delta(0)),
        std::cos(delta(0));
    return SpecialOrthogonalGroup2(m_ * m_delta);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * Details see @ref ManifoldBase::operator-.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x} \in \mathbb{R}^n\f$
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const SpecialOrthogonalGroup2& x) const
  {
    RU_ASSERT(
        x.dim() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        x.dim());

    Matrix2<T> m_delta = x.m_.transpose() * m_;
    HomeSpace delta(1);
    delta(0) = std::atan2(m_delta(1, 0), m_delta(0, 0));
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  /**
   * @brief Get the identity of the special orthogonal group
   * @return The identity of the special orthogonal group, \f$\mathrm{x} =
   * \mathbf{I}_2\f$
   */
  SpecialOrthogonalGroup2<T> getIdentity(void) const
  {
    return SpecialOrthogonalGroup2<T>();
  }

 private:
  Data m_;
};

/**
 * @brief Class for special orthogonal group, \f$\mathcal{M} = SO(3)\f$
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class SpecialOrthogonalGroup3 : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "SpecialOrthogonalGroup3 only supports float and double");

 public:
  using Scalar = T;
  using Data = Matrix3<T>;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<SpecialOrthogonalGroup3<T>>;
  using ConstPtr = std::shared_ptr<const SpecialOrthogonalGroup3<T>>;

  /**
   * @brief Constructor of the special orthogonal group
   *
   * This constructor initializes the special orthogonal group to the identity
   * matrix.
   */
  SpecialOrthogonalGroup3(void)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup3)
  {
    m_.setIdentity();
  }
  /**
   * @brief Constructor of the special orthogonal group
   * @param[in] m The point in the special orthogonal group, \f$\mathrm{m} \in
   * SO\left(3\right)\f$
   */
  explicit SpecialOrthogonalGroup3(const Data& m)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup3)
  {
    m_ = m;
  }
  virtual ~SpecialOrthogonalGroup3(void) = default;

  /**
   * @brief Get the dimension of the special orthogonal group
   * @return The dimension of the special orthogonal group
   */
  virtual size_t dim(void) const override { return 3; }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    const SpecialOrthogonalGroup3<T>& y_t =
        (const SpecialOrthogonalGroup3<T>&)y;

    Eigen::AngleAxis<T> angle_axis(m_.transpose() * y_t.m_);
    Vector3<T> delta = angle_axis.angle() * angle_axis.axis();
    return delta;
  }

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    SpecialOrthogonalGroup3<T>& y_t = (SpecialOrthogonalGroup3<T>&)y;

    Matrix3<T> dR;
    if (phi.norm() < kEpsilon) {
      Matrix3<T> skew_phi = Vec2Skew<T>({phi(0), phi(1), phi(2)});
      dR = Matrix3<T>::Identity() + skew_phi + skew_phi * skew_phi / 2;
    } else {
      Eigen::AngleAxis<T> angle_axis;
      angle_axis.angle() = phi.norm();
      angle_axis.axis() = phi.normalized();
      dR = angle_axis.toRotationMatrix();
    }
    y_t.m_ = m_ * dR;
  }

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * Details see @ref ManifoldBase::add.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @return The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$
   */
  SpecialOrthogonalGroup3 operator+(const HomeSpace& delta) const
  {
    RU_ASSERT(
        delta.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        delta.size());

    Matrix3<T> dR;
    if (delta.norm() < kEpsilon) {
      Matrix3<T> skew_delta =
          Vec2Skew<T>({delta(0), delta(1), delta(2)});
      dR = Matrix3<T>::Identity() + skew_delta +
           skew_delta * skew_delta / 2;
    } else {
      Eigen::AngleAxis<T> angle_axis;
      angle_axis.angle() = delta.norm();
      angle_axis.axis() = delta.normalized();
      dR = angle_axis.toRotationMatrix();
    }
    return SpecialOrthogonalGroup3(m_ * dR);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * Details see @ref ManifoldBase::operator-.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x} \in \mathbb{R}^n\f$
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const SpecialOrthogonalGroup3& x) const
  {
    RU_ASSERT(
        x.dim() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        x.dim());

    Eigen::AngleAxis<T> angle_axis(x.m_.transpose() * m_);
    Vector3<T> delta = angle_axis.angle() * angle_axis.axis();
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  /**
   * @brief Get the identity of the special orthogonal group
   * @return The identity of the special orthogonal group, \f$\mathrm{x} =
   * \mathbf{I}_3\f$
   */
  SpecialOrthogonalGroup3<T> getIdentity(void) const
  {
    return SpecialOrthogonalGroup3<T>();
  }

 private:
  static constexpr T kEpsilon = std::is_same_v<T, float> ? 1e-6f : 1e-8;

  Data m_;
};

/**
 * @brief Class for 2-D surface, \f$\mathcal{M} = \mathcal{S}, \mathcal{S}
 * \doteq
 * \left\{\mathrm{x} \in \mathbb{R}^3 | z = F\left(x, y\right)\right\}\f$
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class Surface2D : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "Surface2D only supports float and double");

 public:
  using Scalar = T;
  using Data = Vector3<T>;
  /**
   * @brief Function type for the surface function \f$z = f(x, y)\f$
   * @param[in] x: The x coordinate, \f$x\f$
   * @param[in] y: The y coordinate, \f$y\f$
   * @return The z coordinate, \f$z = f(x, y)\f$
   */
  using SurfFunc = std::function<T(T, T)>;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<Surface2D<T>>;
  using ConstPtr = std::shared_ptr<const Surface2D<T>>;

  /**
   * @brief Constructor of the 2-D surface
   *
   * This constructor initializes the 2-D surface to the identity point
   * \f$(0, 0, f(0, 0))\f$.
   *
   * @param[in] f The surface function \f$z = f(x, y)\f$
   */
  explicit Surface2D(const SurfFunc& f)
      : ManifoldBase<T>(ManifoldType::kSurface2D), f_(f)
  {
    m_.setZero();
    m_(2) = f_(m_(0), m_(1));
  }
  /**
   * @brief Constructor of the 2-D surface
   * @param[in] f The surface function \f$z = f(x, y)\f$
   * @param[in] m The point in the 2-D surface, \f$\mathrm{m} \in \mathcal{S}\f$
   * (the z coordinate of \f$\mathrm{m}\f$ will be updated to
   * \f$f(m(0), m(1))\f$)
   */
  Surface2D(const SurfFunc& f, const Data& m)
      : ManifoldBase<T>(ManifoldType::kSurface2D), m_(m), f_(f)
  {
    m_(2) = f_(m_(0), m_(1));
  }
  virtual ~Surface2D(void) = default;

  /**
   * @brief Get the dimension of the 2-D surface
   * @return The dimension of the 2-D surface
   */
  virtual size_t dim(void) const override { return 2; }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    const Surface2D<T>& y_t = (const Surface2D<T>&)y;

    return (y_t.m_ - m_).head(2);
  }

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const override
  {
    RU_ASSERT(ManifoldBase<T>::isSame(y), "Type error");

    Surface2D<T>& y_t = (Surface2D<T>&)y;

    y_t.m_.head(2) = m_.head(2) + phi;
    y_t.m_(2) = f_(y_t.m_(0), y_t.m_(1));
  }

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * Details see @ref ManifoldBase::add.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @return The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$
   */
  Surface2D operator+(const HomeSpace& delta) const
  {
    RU_ASSERT(delta.size() == 2,
              "Surface2D dimension mismatch, expected %zu, got %zu", 2,
              delta.size());

    Data m = m_;
    m.head(2) += delta;
    return Surface2D(f_, m);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * Details see @ref ManifoldBase::operator-.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x} \in \mathbb{R}^n\f$
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const Surface2D& x) const { return (m_ - x.m_).head(2); }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  SurfFunc& func(void) { return f_; }

  /**
   * @brief Get the identity of the 2-D surface
   * @return The identity of the 2-D surface, \f$\mathrm{x} = (0, 0, f(0, 0))\f$
   */
  Surface2D<T> getIdentity(void) const { return Surface2D<T>(f_); }

 private:
  Data m_;
  SurfFunc f_;
};

/**
 * @brief Class for compound manifold, which is the cartesian product of
 * arbitrary number of primitive manifolds
 * @tparam T Type of the data (only provides float and double)
 */
template <typename T>
class CompoundManifold : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CompoundManifold only supports float and double");

 public:
  using Scalar = T;
  using ManifoldBasePtr = ManifoldBase<T>::Ptr;
  using Data = std::vector<ManifoldBasePtr>;
  using HomeSpace = VectorX<T>;
  using Ptr = std::shared_ptr<CompoundManifold<T>>;
  using ConstPtr = std::shared_ptr<const CompoundManifold<T>>;

  /**
   * @brief Constructor of the compound manifold
   * @param[in] m: The point in the compound manifold
   */
  explicit CompoundManifold(const Data& m)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold), m_(m)
  {
    RU_ASSERT(!m_.empty(), "CompoundManifold must not be empty");

    for (const auto& prim_m : m_) {
      dim_ += prim_m->dim();
    }
  }
  /**
   * @brief Constructor of the compound manifold
   * @param[in] other: The other compound manifold
   */
  CompoundManifold(const CompoundManifold<T>& other)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold), dim_(other.dim_)
  {
    for (const auto& prim_m : other.m_) {
      ManifoldBasePtr new_prim_m;
      switch (prim_m->type()) {
        case ManifoldType::kEuclideanSpaceX: {
          EuclideanSpaceX<T>* prim_m_t = (EuclideanSpaceX<T>*)(prim_m.get());
          new_prim_m = std::make_shared<EuclideanSpaceX<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSpecialOrthogonalGroup2: {
          SpecialOrthogonalGroup2<T>* prim_m_t =
              (SpecialOrthogonalGroup2<T>*)(prim_m.get());
          new_prim_m = std::make_shared<SpecialOrthogonalGroup2<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSpecialOrthogonalGroup3: {
          SpecialOrthogonalGroup3<T>* prim_m_t =
              (SpecialOrthogonalGroup3<T>*)(prim_m.get());
          new_prim_m = std::make_shared<SpecialOrthogonalGroup3<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSurface2D: {
          Surface2D<T>* prim_m_t = (Surface2D<T>*)(prim_m.get());
          new_prim_m = std::make_shared<Surface2D<T>>(*prim_m_t);
        } break;
        default:
          RU_ASSERT(false, "Unsupported manifold type");
          break;
      }
      m_.push_back(new_prim_m);
    }
  }
  /**
   * @brief Move constructor of the compound manifold
   * @param[in] other: The other compound manifold
   */
  CompoundManifold(const CompoundManifold<T>&& other)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold),
        m_(std::move(other.m_)),
        dim_(other.dim_)
  {
  }
  /**
   * @brief Assignment operator of the compound manifold
   * @param[in] other: The other compound manifold
   * @note When other belongs to the same type as this, is better to use
   * @ref copyData instead of this operator.
   */
  CompoundManifold<T>& operator=(const CompoundManifold<T>& other)
  {
    if (&other == this) {
      return *this;
    }

    dim_ = other.dim_;
    m_.clear();
    for (const auto& prim_m : other.m_) {
      ManifoldBasePtr new_prim_m;
      switch (prim_m->type()) {
        case ManifoldType::kEuclideanSpaceX: {
          EuclideanSpaceX<T>* prim_m_t = (EuclideanSpaceX<T>*)(prim_m.get());
          new_prim_m = std::make_shared<EuclideanSpaceX<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSpecialOrthogonalGroup2: {
          SpecialOrthogonalGroup2<T>* prim_m_t =
              (SpecialOrthogonalGroup2<T>*)(prim_m.get());
          new_prim_m = std::make_shared<SpecialOrthogonalGroup2<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSpecialOrthogonalGroup3: {
          SpecialOrthogonalGroup3<T>* prim_m_t =
              (SpecialOrthogonalGroup3<T>*)(prim_m.get());
          new_prim_m = std::make_shared<SpecialOrthogonalGroup3<T>>(*prim_m_t);
        } break;
        case ManifoldType::kSurface2D: {
          Surface2D<T>* prim_m_t = (Surface2D<T>*)(prim_m.get());
          new_prim_m = std::make_shared<Surface2D<T>>(*prim_m_t);
        } break;
        default:
          RU_ASSERT(false, "Unsupported manifold type");
          break;
      }
      m_.push_back(new_prim_m);
    }

    return *this;
  }
  /**
   * @brief Move assignment operator of the compound manifold
   * @param[in] other: The other compound manifold
   */
  CompoundManifold<T>& operator=(CompoundManifold<T>&& other)
  {
    if (&other == this) {
      return *this;
    }

    dim_ = other.dim_;
    m_ = std::move(other.m_);

    return *this;
  }
  virtual ~CompoundManifold(void) = default;

  /**
   * @brief Get the dimension of the compound manifold
   * @return The dimension of the compound manifold, \f$n\f$
   */
  virtual size_t dim(void) const override { return dim_; }

  /**
   * @brief Check if the manifold is belongs to the same manifold
   * @param[in] other: The other manifold
   * @return true if the manifold is the same, false otherwise
   */
  virtual bool isSame(const ManifoldBase<T>& other) const override
  {
    if (other.type() != ManifoldType::kCompoundManifold) {
      return false;
    }

    const CompoundManifold<T>& other_t = (const CompoundManifold<T>&)other;

    if (m_.size() != other_t.m_.size()) {
      return false;
    }

    for (size_t i = 0; i < m_.size(); ++i) {
      if (!m_[i]->isSame(*(other_t.m_[i]))) {
        return false;
      }
    }

    return true;
  }

  /**
   * @brief Get the local coordinate \f$\varphi\f$ of \f$\mathrm{y}\f$ at point
   * \f$\mathrm{x}\f$
   * @param[in] y: A point in the neighborhood of \f$\mathrm{x}\f$,
   * \f$\mathrm{y} \in U_\mathrm{x}\f$(must belong to the same manifold as this,
   * details see @ref isSame)
   * @return \f$\varphi = \varphi_\mathrm{x}\left(\mathrm{y}\right), \varphi \in
   * \mathbb{R}^n\f$
   */
  virtual HomeSpace proj(const ManifoldBase<T>& y) const override
  {
    RU_ASSERT(y.dim() == dim_,
              "CompoundManifold dimension mismatch, expected %zu, got %zu",
              dim_, y.dim());

    const CompoundManifold<T>& y_t = (const CompoundManifold<T>&)y;

    HomeSpace delta(dim_);
    size_t start_idx = 0;
    for (size_t i = 0; i < m_.size(); ++i) {
      RU_ASSERT(m_[i]->isSame(*y_t.m_[i]),
                "Primitive manifolds are not the same");
      delta.segment(start_idx, m_[i]->dim()) = m_[i]->proj(*(y_t.m_[i]));
      start_idx += m_[i]->dim();
    }
    return delta;
  }

  /**
   * @brief The inverse of the projection map @ref proj
   * @param[in] phi: The local coordinate, \f$\varphi \in \mathbb{R}^n\f$
   * @param[out] y: The point in manifold corresponding to \f$\varphi\f$,
   * \f$\mathrm{y} = \varphi_\mathrm{x}^{-1}\left(\varphi\right)\f$(must belong
   * to the same manifold as this, details see @ref isSame)
   */
  virtual void invProj(const HomeSpace& phi, ManifoldBase<T>& y) const override
  {
    RU_ASSERT(y.dim() == dim_,
              "CompoundManifold dimension mismatch, expected %zu, got %zu",
              dim_, y.dim());

    CompoundManifold<T>& y_t = (CompoundManifold<T>&)y;

    size_t start_idx = 0;
    for (size_t i = 0; i < m_.size(); ++i) {
      RU_ASSERT(m_[i]->isSame(*y_t.m_[i]),
                "Primitive manifolds are not the same");
      m_[i]->invProj(phi.segment(start_idx, m_[i]->dim()), *y_t.m_[i]);
      start_idx += m_[i]->dim();
    }
  }

  /**
   * @brief Function of symbolic operations \f$\boxplus\f$
   *
   * Details see @ref ManifoldBase::add.
   *
   * @param[in] delta: The perturbation in the homeomorphic space, \f$\delta \in
   * \mathbb{R}^n\f$
   * @return The result point on the manifold, \f$\mathrm{y} = \mathrm{x}
   * \boxplus \delta\f$
   */
  CompoundManifold operator+(const HomeSpace& delta) const
  {
    RU_ASSERT(delta.size() == dim_,
              "CompoundManifold dimension mismatch, expected %zu, got %zu",
              dim_, delta.size());

    Data new_m;
    new_m.reserve(m_.size());
    size_t start_idx = 0;
    for (const auto& prim_m : m_) {
      ManifoldBasePtr new_prim_m;
      switch (prim_m->type()) {
        case ManifoldType::kEuclideanSpaceX:
          new_prim_m = std::make_shared<EuclideanSpaceX<T>>(prim_m->dim());
          break;
        case ManifoldType::kSpecialOrthogonalGroup2:
          new_prim_m = std::make_shared<SpecialOrthogonalGroup2<T>>();
          break;
        case ManifoldType::kSpecialOrthogonalGroup3:
          new_prim_m = std::make_shared<SpecialOrthogonalGroup3<T>>();
          break;
        case ManifoldType::kSurface2D: {
          Surface2D<T>* prim_m_t = (Surface2D<T>*)(prim_m.get());
          new_prim_m = std::make_shared<Surface2D<T>>(prim_m_t->func());
        } break;
        default:
          RU_ASSERT(false, "Unsupported manifold type");
          break;
      }
      prim_m->add(delta.segment(start_idx, prim_m->dim()), *new_prim_m);
      new_m.push_back(new_prim_m);
      start_idx += prim_m->dim();
    }

    return CompoundManifold(new_m);
  }

  /**
   * @brief Function of symbolic operations \f$\boxminus\f$
   *
   * Details see @ref ManifoldBase::operator-.
   *
   * @param[in] x: The point on the manifold, \f$\mathrm{x} \in \mathbb{R}^n\f$
   * @return The difference in the homeomorphic space, \f$\mathrm{y} \boxminus
   * \mathrm{x} \in \mathbb{R}^n\f$
   */
  HomeSpace operator-(const CompoundManifold& x) const
  {
    RU_ASSERT(x.dim() == dim_,
              "CompoundManifold dimension mismatch, expected %zu, got %zu",
              dim_, x.dim());

    HomeSpace delta(dim_);
    size_t start_idx = 0;
    for (size_t i = 0; i < m_.size(); ++i) {
      RU_ASSERT(m_[i]->isSame(*x.m_[i]),
                "Primitive manifolds are not the same");
      delta.segment(start_idx, m_[i]->dim()) = *(m_[i]) - *(x.m_[i]);
      start_idx += m_[i]->dim();
    }
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  /**
   * @brief Get the identity of the compound manifold
   * @return The identity of the compound manifold
   */
  CompoundManifold<T> getIdentity(void) const
  {
    Data new_m;
    new_m.reserve(m_.size());
    for (const auto& prim_m : m_) {
      ManifoldBasePtr new_prim_m;
      switch (prim_m->type()) {
        case ManifoldType::kEuclideanSpaceX:
          new_prim_m = std::make_shared<EuclideanSpaceX<T>>(prim_m->dim());
          break;
        case ManifoldType::kSpecialOrthogonalGroup2:
          new_prim_m = std::make_shared<SpecialOrthogonalGroup2<T>>();
          break;
        case ManifoldType::kSpecialOrthogonalGroup3:
          new_prim_m = std::make_shared<SpecialOrthogonalGroup3<T>>();
          break;
        case ManifoldType::kSurface2D: {
          Surface2D<T>* prim_m_t = (Surface2D<T>*)(prim_m.get());
          new_prim_m = std::make_shared<Surface2D<T>>(prim_m_t->func());
        } break;
        default:
          RU_ASSERT(false, "Unsupported manifold type");
          break;
      }
      new_m.push_back(new_prim_m);
    }

    return CompoundManifold<T>(new_m);
  }

  /**
   * @brief Copy data from another compound manifold
   * @param[in] other: The other compound manifold(must belong to the same
   * manifold as this, details see @ref isSame)
   */
  void copyData(const CompoundManifold<T>& other)
  {
    RU_ASSERT(other.dim() == dim_,
              "CompoundManifold dimension mismatch, expected %zu, got %zu",
              dim_, other.dim());

    for (size_t i = 0; i < m_.size(); ++i) {
      RU_ASSERT(m_[i]->isSame(*other.m_[i]),
                "Primitive manifolds are not the same");

      switch (m_[i]->type()) {
        case ManifoldType::kEuclideanSpaceX: {
          EuclideanSpaceX<T>* prim_m_t = (EuclideanSpaceX<T>*)(m_[i].get());
          const EuclideanSpaceX<T>* other_prim_m_t =
              (const EuclideanSpaceX<T>*)(other.m_[i].get());
          prim_m_t->data() = other_prim_m_t->data();
        } break;
        case ManifoldType::kSpecialOrthogonalGroup2: {
          SpecialOrthogonalGroup2<T>* prim_m_t =
              (SpecialOrthogonalGroup2<T>*)(m_[i].get());
          const SpecialOrthogonalGroup2<T>* other_prim_m_t =
              (const SpecialOrthogonalGroup2<T>*)(other.m_[i].get());
          prim_m_t->data() = other_prim_m_t->data();
        } break;
        case ManifoldType::kSpecialOrthogonalGroup3: {
          SpecialOrthogonalGroup3<T>* prim_m_t =
              (SpecialOrthogonalGroup3<T>*)(m_[i].get());
          const SpecialOrthogonalGroup3<T>* other_prim_m_t =
              (const SpecialOrthogonalGroup3<T>*)(other.m_[i].get());
          prim_m_t->data() = other_prim_m_t->data();
        } break;
        case ManifoldType::kSurface2D: {
          Surface2D<T>* prim_m_t = (Surface2D<T>*)(m_[i].get());
          const Surface2D<T>* other_prim_m_t =
              (const Surface2D<T>*)(other.m_[i].get());
          prim_m_t->data() = other_prim_m_t->data();
        } break;
        default:
          RU_ASSERT(false, "Unsupported manifold type");
          break;
      }
    }
  }

 private:
  Data m_;
  size_t dim_ = 0;
};

template class ManifoldBase<float>;
using ManifoldBasef = ManifoldBase<float>;
template class ManifoldBase<double>;
using ManifoldBased = ManifoldBase<double>;
template class EuclideanSpaceX<float>;
using EuclideanSpaceXf = EuclideanSpaceX<float>;
template class EuclideanSpaceX<double>;
using EuclideanSpaceXd = EuclideanSpaceX<double>;
template class SpecialOrthogonalGroup2<float>;
using SpecialOrthogonalGroup2f = SpecialOrthogonalGroup2<float>;
template class SpecialOrthogonalGroup2<double>;
using SpecialOrthogonalGroup2d = SpecialOrthogonalGroup2<double>;
template class SpecialOrthogonalGroup3<float>;
using SpecialOrthogonalGroup3f = SpecialOrthogonalGroup3<float>;
template class SpecialOrthogonalGroup3<double>;
using SpecialOrthogonalGroup3d = SpecialOrthogonalGroup3<double>;
template class Surface2D<float>;
using Surface2Df = Surface2D<float>;
template class Surface2D<double>;
using Surface2Dd = Surface2D<double>;
template class CompoundManifold<float>;
using CompoundManifoldf = CompoundManifold<float>;
template class CompoundManifold<double>;
using CompoundManifoldd = CompoundManifold<double>;
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
}  // namespace robot_utils

#endif /* ROBOT_UTILS_GEOMETRY_MANIFOLD_HPP_ */
