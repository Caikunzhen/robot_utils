/**
 *******************************************************************************
 * @file      : manifold.hpp
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
#ifndef ROBOT_UTILS_GEOMETRY_MANIFOLD_HPP_
#define ROBOT_UTILS_GEOMETRY_MANIFOLD_HPP_

/* Includes ------------------------------------------------------------------*/
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <type_traits>
#include <variant>
#include <vector>

#include "robot_utils/core/assert.hpp"
#include "robot_utils/core/periodic_data.hpp"
/* Exported macro ------------------------------------------------------------*/

namespace robot_utils
{
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
enum class ManifoldType {
  kEuclideanSpaceX,
  kSpecialOrthogonalGroup2,
  kSpecialOrthogonalGroup3,
  kSurface2D,
  kCompoundManifold
};

template <typename T>
class ManifoldBase
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "ManifoldBase only supports float and double");

 public:
  using Scalar = T;
  using Type = ManifoldType;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<ManifoldBase<T>>;
  using ConstPtr = std::shared_ptr<const ManifoldBase<T>>;

  explicit ManifoldBase(Type type) : type_(type) {}
  virtual ~ManifoldBase(void) = default;

  virtual size_t dim(void) const = 0;

  Type type(void) const { return type_; }

  virtual bool isSame(const ManifoldBase<T>& other) const
  {
    return type_ == other.type_ && dim() == other.dim();
  }

  virtual HomeSpace toHomeSpace(void) const = 0;
  virtual void fromHomeSpace(const HomeSpace& x) = 0;

  virtual ManifoldBase<T>& operator+=(const HomeSpace& delta) = 0;
  virtual ManifoldBase<T>& operator-=(const HomeSpace& delta) = 0;

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const = 0;
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const = 0;

 protected:
  const Type type_;
};

template <typename T>
class EuclideanSpaceX : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "EuclideanSpaceX only supports float and double");

 public:
  using Scalar = T;
  using Data = Eigen::VectorX<T>;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<EuclideanSpaceX<T>>;
  using ConstPtr = std::shared_ptr<const EuclideanSpaceX<T>>;

  explicit EuclideanSpaceX(size_t dim)
      : ManifoldBase<T>(ManifoldType::kEuclideanSpaceX), dim_(dim)
  {
    m_.resize(dim);
    m_.setZero();
  }
  explicit EuclideanSpaceX(const HomeSpace& x)
      : ManifoldBase<T>(ManifoldType::kEuclideanSpaceX)
  {
    dim_ = x.size();
    m_ = x;
  }
  virtual ~EuclideanSpaceX(void) = default;

  virtual size_t dim(void) const override { return dim_; }

  virtual HomeSpace toHomeSpace(void) const override { return m_; }
  virtual void fromHomeSpace(const HomeSpace& x) override
  {
    PARAM_ASSERT(x.size() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, x.size());

    m_ = x;
  }

  virtual EuclideanSpaceX& operator+=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    m_ += delta;
    return *this;
  }
  virtual EuclideanSpaceX& operator-=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    m_ -= delta;
    return *this;
  }

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());
    PARAM_ASSERT(ManifoldBase<T>::isSame(out), "Type error");

    EuclideanSpaceX<T>& out_t = (EuclideanSpaceX<T>&)out;

    out_t.m_ = m_ + delta;
  }
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const override
  {
    PARAM_ASSERT(ManifoldBase<T>::isSame(x), "Type error");

    EuclideanSpaceX<T>& x_t = (EuclideanSpaceX<T>&)x;

    return m_ - x_t.m_;
  }
  EuclideanSpaceX operator+(const HomeSpace& delta) const
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    return EuclideanSpaceX(m_ + delta);
  }
  HomeSpace operator-(const EuclideanSpaceX& x) const
  {
    PARAM_ASSERT(x.dim() == dim_,
                 "EuclideanSpaceX dimension mismatch, expected %zu, got %zu",
                 dim_, x.dim());

    return m_ - x.m_;
  }

  Data& getData(void) { return m_; }
  const Data& data(void) const { return m_; }

  EuclideanSpaceX<T> getIdentity(void) const
  {
    return EuclideanSpaceX<T>(dim_);
  }

 private:
  Data m_;
  size_t dim_ = 0;
};

template <typename T>
class SpecialOrthogonalGroup2 : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "SpecialOrthogonalGroup2 only supports float and double");

 public:
  using Scalar = T;
  using Data = Eigen::Matrix2<T>;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<SpecialOrthogonalGroup2<T>>;
  using ConstPtr = std::shared_ptr<const SpecialOrthogonalGroup2<T>>;

  SpecialOrthogonalGroup2(void)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup2)
  {
    m_.setIdentity();
  }
  explicit SpecialOrthogonalGroup2(const HomeSpace& x)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup2)
  {
    fromHomeSpace(x);
  }
  explicit SpecialOrthogonalGroup2(const Data& m)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup2)
  {
    m_ = m;
  }
  virtual ~SpecialOrthogonalGroup2(void) = default;

  virtual size_t dim(void) const override { return 1; }

  virtual HomeSpace toHomeSpace(void) const override
  {
    HomeSpace x(1);
    x(0) = atan2(m_(1, 0), m_(0, 0));
    return x;
  }
  virtual void fromHomeSpace(const HomeSpace& x) override
  {
    PARAM_ASSERT(
        x.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        x.size());

    m_ << cos(x(0)), -sin(x(0)), sin(x(0)), cos(x(0));
  }

  virtual SpecialOrthogonalGroup2& operator+=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(
        delta.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        delta.size());

    Eigen::Matrix2<T> m_delta;
    m_delta << cos(delta(0)), -sin(delta(0)), sin(delta(0)), cos(delta(0));
    m_ *= m_delta;
    return *this;
  }
  virtual SpecialOrthogonalGroup2& operator-=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(
        delta.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        delta.size());

    Eigen::Matrix2<T> m_delta;
    m_delta << cos(delta(0)), sin(delta(0)), -sin(delta(0)), cos(delta(0));
    m_ *= m_delta;
    return *this;
  }

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const override
  {
    PARAM_ASSERT(
        delta.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        delta.size());
    PARAM_ASSERT(ManifoldBase<T>::isSame(out), "Type error");

    SpecialOrthogonalGroup2<T>& out_t = (SpecialOrthogonalGroup2<T>&)out;

    Eigen::Matrix2<T> m_delta;
    m_delta << cos(delta(0)), -sin(delta(0)), sin(delta(0)), cos(delta(0));
    out_t.m_ = m_ * m_delta;
  }
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const override
  {
    PARAM_ASSERT(ManifoldBase<T>::isSame(x), "Type error");

    SpecialOrthogonalGroup2<T>& x_t = (SpecialOrthogonalGroup2<T>&)x;

    Eigen::Matrix2<T> m_delta = m_.transpose() * x_t.m_;
    HomeSpace delta(1);
    delta(0) = atan2(m_delta(1, 0), m_delta(0, 0));
    AngleRadNorm(&delta(0));
    return delta;
  }
  SpecialOrthogonalGroup2 operator+(const HomeSpace& delta) const
  {
    PARAM_ASSERT(
        delta.size() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        delta.size());

    HomeSpace x(1);
    x(0) = atan2(m_(1, 0), m_(0, 0)) + delta(0);
    return SpecialOrthogonalGroup2(x);
  }

  HomeSpace operator-(const SpecialOrthogonalGroup2& x) const
  {
    PARAM_ASSERT(
        x.dim() == 1,
        "SpecialOrthogonalGroup2 dimension mismatch, expected %zu, got %zu", 1,
        x.dim());

    HomeSpace delta(1);
    delta(0) = atan2(m_(1, 0), m_(0, 0)) - atan2(x.m_(1, 0), x.m_(0, 0));
    AngleRadNorm(&delta(0));
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  SpecialOrthogonalGroup2<T> getIdentity(void) const
  {
    return SpecialOrthogonalGroup2<T>();
  }

 private:
  Data m_;
};

template <typename T>
class SpecialOrthogonalGroup3 : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "SpecialOrthogonalGroup3 only supports float and double");

 public:
  using Scalar = T;
  using Data = Eigen::Matrix3<T>;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<SpecialOrthogonalGroup3<T>>;
  using ConstPtr = std::shared_ptr<const SpecialOrthogonalGroup3<T>>;

  SpecialOrthogonalGroup3(void)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup3)
  {
    m_.setIdentity();
  }
  explicit SpecialOrthogonalGroup3(const HomeSpace& x)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup3)
  {
    fromHomeSpace(x);
  }
  explicit SpecialOrthogonalGroup3(const Data& m)
      : ManifoldBase<T>(ManifoldType::kSpecialOrthogonalGroup3)
  {
    m_ = m;
  }
  virtual ~SpecialOrthogonalGroup3(void) = default;

  virtual size_t dim(void) const override { return 3; }

  virtual HomeSpace toHomeSpace(void) const override
  {
    Eigen::AngleAxis<T> angle_axis(m_);
    Eigen::Vector3<T> v = angle_axis.angle() * angle_axis.axis();
    HomeSpace x(3);
    x << v(0), v(1), v(2);
    return x;
  }
  virtual void fromHomeSpace(const HomeSpace& x) override
  {
    PARAM_ASSERT(
        x.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        x.size());

    Eigen::AngleAxis<T> angle_axis;
    angle_axis.angle() = x.norm();
    angle_axis.axis() = x / angle_axis.angle();
    m_ = angle_axis.toRotationMatrix();
  }

  virtual SpecialOrthogonalGroup3& operator+=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(
        delta.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        delta.size());

    Eigen::AngleAxis<T> angle_axis;
    angle_axis.angle() = delta.norm();
    angle_axis.axis() = delta / angle_axis.angle();
    Eigen::Matrix3<T> m_delta = angle_axis.toRotationMatrix();
    m_ *= m_delta;
    return *this;
  }
  virtual SpecialOrthogonalGroup3& operator-=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(
        delta.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        delta.size());

    Eigen::AngleAxis<T> angle_axis;
    angle_axis.angle() = delta.norm();
    angle_axis.axis() = delta / angle_axis.angle();
    Eigen::Matrix3<T> m_delta = angle_axis.toRotationMatrix().transpose();
    m_ *= m_delta;
    return *this;
  }

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const override
  {
    PARAM_ASSERT(
        delta.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        delta.size());
    PARAM_ASSERT(ManifoldBase<T>::isSame(out), "Type error");

    SpecialOrthogonalGroup3<T>& out_t = (SpecialOrthogonalGroup3<T>&)out;

    Eigen::AngleAxis<T> angle_axis;
    angle_axis.angle() = delta.norm();
    angle_axis.axis() = delta / angle_axis.angle();
    Eigen::Matrix3<T> m_delta = angle_axis.toRotationMatrix();
    out_t.m_ = m_ * m_delta;
  }
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const override
  {
    PARAM_ASSERT(ManifoldBase<T>::isSame(x), "Type error");

    SpecialOrthogonalGroup3<T>& x_t = (SpecialOrthogonalGroup3<T>&)x;

    Eigen::AngleAxis<T> angle_axis(m_.transpose() * x_t.m_);
    Eigen::Vector3<T> v = angle_axis.angle() * angle_axis.axis();
    HomeSpace delta(3);
    delta << v(0), v(1), v(2);
    AngleRadNorm(&delta(0));
    return delta;
  }
  SpecialOrthogonalGroup3 operator+(const HomeSpace& delta) const
  {
    PARAM_ASSERT(
        delta.size() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        delta.size());

    Eigen::AngleAxis<T> angle_axis;
    angle_axis.angle() = delta.norm();
    angle_axis.axis() = delta / angle_axis.angle();
    Eigen::Matrix3<T> m_delta = angle_axis.toRotationMatrix();
    return SpecialOrthogonalGroup3(Data(m_ * m_delta));
  }
  HomeSpace operator-(const SpecialOrthogonalGroup3& x) const
  {
    PARAM_ASSERT(
        x.dim() == 3,
        "SpecialOrthogonalGroup3 dimension mismatch, expected %zu, got %zu", 3,
        x.dim());

    Eigen::AngleAxis<T> angle_axis(m_.transpose() * x.m_);
    Eigen::Vector3<T> v = angle_axis.angle() * angle_axis.axis();
    HomeSpace delta(3);
    delta << v(0), v(1), v(2);
    AngleRadNorm(&delta(0));
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  SpecialOrthogonalGroup3<T> getIdentity(void) const
  {
    return SpecialOrthogonalGroup3<T>();
  }

 private:
  Data m_;
};

template <typename T>
class Surface2D : public ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "Surface2D only supports float and double");

 public:
  using Scalar = T;
  using Data = Eigen::Vector3<T>;
  /**
   * @brief Function type for the surface function f(x, y) = z
   * @param x
   * @param y
   * @return z
   */
  using SurfFunc = std::function<T(T, T)>;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<Surface2D<T>>;
  using ConstPtr = std::shared_ptr<const Surface2D<T>>;

  explicit Surface2D(const SurfFunc& f)
      : ManifoldBase<T>(ManifoldType::kSurface2D), f_(f)
  {
    m_.setZero();
    m_(2) = f_(m_(0), m_(1));
  }
  Surface2D(const SurfFunc f, const HomeSpace& x)
      : ManifoldBase<T>(ManifoldType::kSurface2D), f_(f)
  {
    fromHomeSpace(x);
  }
  Surface2D(const SurfFunc& f, const Data& m)
      : ManifoldBase<T>(ManifoldType::kSurface2D), f_(f), m_(m)
  {
    m_(2) = f_(m_(0), m_(1));
  }
  virtual ~Surface2D(void) = default;

  virtual size_t dim(void) const override { return 2; }

  virtual HomeSpace toHomeSpace(void) const override { return m_.head(2); }
  virtual void fromHomeSpace(const HomeSpace& x) override
  {
    PARAM_ASSERT(x.size() == 2,
                 "Surface2D dimension mismatch, expected %zu, got %zu", 2,
                 x.size());

    m_.head(2) = x;
    m_(2) = f_(m_(0), m_(1));
  }

  virtual Surface2D& operator+=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == 2,
                 "Surface2D dimension mismatch, expected %zu, got %zu", 2,
                 delta.size());

    m_.head(2) += delta;
    m_(2) = f_(m_(0), m_(1));
    return *this;
  }
  virtual Surface2D& operator-=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == 2,
                 "Surface2D dimension mismatch, expected %zu, got %zu", 2,
                 delta.size());

    m_.head(2) -= delta;
    m_(2) = f_(m_(0), m_(1));
    return *this;
  }

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const override
  {
    PARAM_ASSERT(delta.size() == 2,
                 "Surface2D dimension mismatch, expected %zu, got %zu", 2,
                 delta.size());
    PARAM_ASSERT(ManifoldBase<T>::isSame(out), "Type error");

    Surface2D<T>& out_t = (Surface2D<T>&)out;

    out_t.m_.head(2) = m_.head(2) + delta;
    out_t.m_(2) = f_(out_t.m_(0), out_t.m_(1));
  }
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const override
  {
    PARAM_ASSERT(ManifoldBase<T>::isSame(x), "Type error");

    Surface2D<T>& x_t = (Surface2D<T>&)x;

    return (m_ - x_t.m_).head(2);
  }
  Surface2D operator+(const HomeSpace& delta) const
  {
    PARAM_ASSERT(delta.size() == 2,
                 "Surface2D dimension mismatch, expected %zu, got %zu", 2,
                 delta.size());

    return Surface2D(f_, HomeSpace(m_.head(2) + delta));
  }
  HomeSpace operator-(const Surface2D& x) const { return (m_ - x.m_).head(2); }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

  SurfFunc& func(void) { return f_; }

  Surface2D<T> getIdentity(void) const { return Surface2D<T>(f_); }

 private:
  Data m_;
  SurfFunc f_;
};

template <typename T>
class CompoundManifold : ManifoldBase<T>
{
  static_assert(std::is_same_v<T, float> || std::is_same_v<T, double>,
                "CompoundManifold only supports float and double");

 public:
  using Scalar = T;
  using ManifoldBasePtr = ManifoldBase<T>::Ptr;
  using Data = std::vector<ManifoldBasePtr>;
  using HomeSpace = Eigen::VectorX<T>;
  using Ptr = std::shared_ptr<CompoundManifold<T>>;
  using ConstPtr = std::shared_ptr<const CompoundManifold<T>>;

  explicit CompoundManifold(const Data& m)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold), m_(m)
  {
    PARAM_ASSERT(!m_.empty(), "CompoundManifold must not be empty");

    for (const auto& prim_m : m_) {
      dim_ += prim_m->dim();
    }
  }
  CompoundManifold(const CompoundManifold<T>& other)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold), dim_(other.dim_)
  {
    size_t start_idx = 0;
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
          PARAM_ASSERT(false, "Unsupported manifold type");
          break;
      }
      m_.push_back(new_prim_m);
      start_idx += prim_m->dim();
    }
  }
  CompoundManifold(const CompoundManifold<T>&& other)
      : ManifoldBase<T>(ManifoldType::kCompoundManifold), dim_(other.dim_)
  {
    m_ = std::move(other.m_);
  }
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
          PARAM_ASSERT(false, "Unsupported manifold type");
          break;
      }
      m_.push_back(new_prim_m);
    }

    return *this;
  }
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

  virtual size_t dim(void) const override { return dim_; }

  virtual HomeSpace toHomeSpace(void) const override
  {
    HomeSpace x(dim_);
    size_t start_idx = 0;
    for (const auto& prim_m : m_) {
      x.segment(start_idx, prim_m->dim()) = prim_m->toHomeSpace();
      start_idx += prim_m->dim();
    }
    return x;
  }
  virtual void fromHomeSpace(const HomeSpace& x) override
  {
    PARAM_ASSERT(x.size() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, x.size());

    size_t start_idx = 0;
    for (auto& prim_m : m_) {
      prim_m->fromHomeSpace(x.segment(start_idx, prim_m->dim()));
      start_idx += prim_m->dim();
    }
  }

  virtual CompoundManifold& operator+=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    size_t start_idx = 0;
    for (auto& prim_m : m_) {
      prim_m->operator+=(delta.segment(start_idx, prim_m->dim()));
      start_idx += prim_m->dim();
    }
    return *this;
  }
  virtual CompoundManifold& operator-=(const HomeSpace& delta) override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    size_t start_idx = 0;
    for (auto& prim_m : m_) {
      prim_m->operator-=(delta.segment(start_idx, prim_m->dim()));
      start_idx += prim_m->dim();
    }
    return *this;
  }

  virtual void add(const HomeSpace& delta, ManifoldBase<T>& out) const override
  {
    PARAM_ASSERT(delta.size() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, delta.size());

    CompoundManifold<T>& out_t = (CompoundManifold<T>&)out;

    size_t start_idx = 0;
    for (size_t i = 0; i < dim_; ++i) {
      PARAM_ASSERT(m_[i]->isSame(*out_t.m_[i]),
                   "Primitive manifolds are not the same");
      m_[i]->add(delta.segment(start_idx, m_[i]->dim()), *out_t.m_[i]);
      start_idx += m_[i]->dim();
    }
  }
  virtual HomeSpace operator-(const ManifoldBase<T>& x) const
  {
    PARAM_ASSERT(x.dim() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, x.dim());

    CompoundManifold<T>& x_t = (CompoundManifold<T>&)x;

    HomeSpace delta(dim_);
    size_t start_idx = 0;
    for (size_t i = 0; i < m_.size(); ++i) {
      PARAM_ASSERT(m_[i]->isSame(*x_t.m_[i]),
                   "Primitive manifolds are not the same");
      delta.segment(start_idx, m_[i]->dim()) = *(m_[i]) - *(x_t.m_[i]);
      start_idx += m_[i]->dim();
    }
    return delta;
  }
  CompoundManifold operator+(const HomeSpace& delta) const
  {
    PARAM_ASSERT(delta.size() == dim_,
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
          PARAM_ASSERT(false, "Unsupported manifold type");
          break;
      }
      prim_m->add(delta.segment(start_idx, prim_m->dim()), *new_prim_m);
      new_m.push_back(new_prim_m);
      start_idx += prim_m->dim();
    }

    return CompoundManifold(new_m);
  }
  HomeSpace operator-(const CompoundManifold& x) const
  {
    PARAM_ASSERT(x.dim() == dim_,
                 "CompoundManifold dimension mismatch, expected %zu, got %zu",
                 dim_, x.dim());

    HomeSpace delta(dim_);
    size_t start_idx = 0;
    for (size_t i = 0; i < m_.size(); ++i) {
      PARAM_ASSERT(m_[i]->isSame(*x.m_[i]),
                   "Primitive manifolds are not the same");
      delta.segment(start_idx, m_[i]->dim()) = *(m_[i]) - *(x.m_[i]);
      start_idx += m_[i]->dim();
    }
    return delta;
  }

  Data& data(void) { return m_; }
  const Data& data(void) const { return m_; }

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
          PARAM_ASSERT(false, "Unsupported manifold type");
          break;
      }
      new_m.push_back(new_prim_m);
    }

    return CompoundManifold<T>(new_m);
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
