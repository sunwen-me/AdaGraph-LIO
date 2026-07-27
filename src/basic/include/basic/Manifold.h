#ifndef Manifold_MATH_H_H_
#define Manifold_MATH_H_H_


#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "alias.h"


namespace BASIC {

template <class scalar>
inline scalar tolerance();

template <>
inline float tolerance<float>() {
    return 1e-6f;
}
template <>
inline double tolerance<double>() {
    return 1e-11;
}

template <typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SKEW(const Eigen::MatrixBase<Derived> &v) {
  Eigen::Matrix<typename Derived::Scalar, 3, 3>  m;
  m << typename Derived::Scalar(0), -v[2], v[1],
      v[2], typename Derived::Scalar(0), -v[0],
      -v[1], v[0], typename Derived::Scalar(0);
  return m;
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang) {
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    /// Roderigous Tranformation
    return Eye3 + std::sin(ang_norm) * K + (T(1.0) - std::cos(ang_norm)) * K * K;
  } else {
    return Eye3;
  }
}

// left Jacobian
template <typename T>
inline Eigen::Matrix<T, 3, 3> Gamma_1(const Eigen::Matrix<T, 3, 1>& ang){
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    return Eye3 + ((T(1.0) - std::cos(ang_norm)) * K + (ang_norm - std::sin(ang_norm)) * K * K) / ang_norm;
  } else {
    return Eye3;
  }
}

// Second-order differential variable
// Gamma_2(phi) = 0.5*I + (theta - sin(theta))/theta^2 * K
//              + (theta^2 + 2*cos(theta) - 2)/(2*theta^2) * K*K,  K = SKEW(phi/theta)
template <typename T>
inline Eigen::Matrix<T, 3, 3> Gamma_2(const Eigen::Matrix<T, 3, 1>& ang){
  T ang_norm = ang.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (ang_norm > 1e-5) {
    Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_axis);
    return T(0.5) * Eye3 + ( T(2.0) * (ang_norm - std::sin(ang_norm)) * K
                          + ((ang_norm * ang_norm + T(2.0) * std::cos(ang_norm) - T(2.0)) * K * K) )
                        / ( T(2.0) * ang_norm * ang_norm);
  } else {
    // series: 0.5*I + SKEW(phi)/3! + O(|phi|^2)
    return T(0.5) * Eye3 + SKEW(ang) / T(6.0);
  }
}

//// SO(3) left Jacobian
M3 A_matrix(const V3& v);

template <typename T, typename Ts>
inline Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt) {
  T ang_vel_norm = ang_vel.norm();
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (ang_vel_norm > 0.0000001) {
    Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix<T, 3, 3> K;

    K = SKEW(r_axis);

    T r_ang = ang_vel_norm * dt;

    return Eye3 + std::sin(r_ang) * K + (T(1.0) - std::cos(r_ang)) * K * K;
  } else {
    return Eye3;
  }
}

template <typename T>
inline Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3) {
  T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
  Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
  if (norm > 1e-10) {
    Eigen::Matrix<T, 3, 1> r_ang = {v1 / norm, v2 / norm, v3 / norm};
    Eigen::Matrix<T, 3, 3> K;
    K = SKEW(r_ang);
    return Eye3 + std::sin(norm) * K + (T(1.0) - std::cos(norm)) * K * K;
  } else {
    return Eye3;
  }
}


/* Logrithm of a Rotation Matrix */
// Quaternion-based (Eigen's matrix-to-quaternion conversion uses the stable
// max-diagonal branch), well-conditioned for all angles including theta ~ pi.
template <typename T>
inline Eigen::Matrix<T, 3, 1> Log(const Eigen::Matrix<T, 3, 3> &R) {
  Eigen::Quaternion<T> q(R);
  q.normalize();
  if (q.w() < T(0)) {
    q.coeffs() *= T(-1);
  }
  const Eigen::Matrix<T, 3, 1> vector_part(q.x(), q.y(), q.z());
  const T sin_half_theta = vector_part.norm();
  if (sin_half_theta < tolerance<T>()) {
    return T(2.0) * vector_part;
  }
  const T theta = T(2.0) * std::atan2(sin_half_theta, q.w());
  return (theta / sin_half_theta) * vector_part;
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot) {
  T sy = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool singular = sy < 1e-6;
  T x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  } else {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<T, 3, 1> ang(x, y, z);
  return ang;
}

inline M4 hat6(const V6 &xi){
  M4 xi_hat;
  xi_hat  <<  0.0,   -xi(2),  xi(1), xi(3),
              xi(2),  0.0,   -xi(0), xi(4),
             -xi(1),  xi(0),  0.0,   xi(5),
              0.0,    0.0,    0.0,   0.0;
  return xi_hat;
}

inline V6 vee6(const M4 &xi_hat){
  V6 xi;
  xi << -xi_hat(1,2), xi_hat(0,2), -xi_hat(0,1),
          xi_hat(0,3), xi_hat(1,3), xi_hat(2,3);
  return xi;
}


/**
 * SO3 and SE3 ref to https://github.com/hku-mars/BALM/blob/master/src/compare_test/SE3/SE3.hpp
 */
class SO3
{
public:
  SO3(const M3 &R = Eye3);

  SO3(const V3 &w);

  SO3(const SO3 &R);

  template<typename OtherDerived>
  SO3(const Eigen::MatrixBase<OtherDerived>& rhs):
  R_(rhs)
  {}

  static M3 hat(const V3 &v);

  static V3 vee(const M3 &w_hat);

  static SO3 Exp(const V3 &ang);

  static SO3 Exp(const V3 &ang_vel, const scalar &dt);

  static M3 Exp_m3(const V3 &ang_vel, const scalar &dt);

  SO3& operator=(const SO3& rhs);

  SO3 operator*(const SO3& rhs) const noexcept;

  SO3 operator*(scalar s) const noexcept;

  V3 operator*(const V3& rhs) const noexcept;

  template<typename OtherDerived>
  V3 operator*(const Eigen::MatrixBase<OtherDerived>& rhs) const noexcept{
    return R_ * rhs;
  }

  void update(const V3 &dw);

  void updateRhs(const V3 &dw);

  void exp(const M3 &w_hat);

  V4 coeffs() const noexcept;

  M3 log(double *ro = nullptr) const noexcept;

  V3 log_vee() const noexcept;

  SO3 inverse(void) const noexcept;

  M3 adjoint() const noexcept;

  M3 R() const noexcept;

  Quat quaternion() const noexcept;

  M3 matrix() const noexcept;

  M3& ref2R();

  scalar distance(const SO3 &rhs) const noexcept;

  void print(void) const noexcept;

  void print_lie(void) const noexcept;

  scalar yaw() const noexcept;

public:
  M3 R_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



class SE3
{
public:
  SE3() : T_(M4::Identity()), R_(M3::Identity()), t_(V3::Zero()) {}

  SE3(const M4 &T);

  SE3(const V6 &xi);
  
  SE3(const SE3 &T);

  SE3(const SO3 &R, const V3 &t);

  SE3(const M3 &R, const V3 &t);

  SE3(const Quat &q, const V3 &t);

  template<typename OtherDerived>
  SE3(const Eigen::MatrixBase<OtherDerived>& rhs)  :
  T_(rhs)
  {
    R_ = T_.topLeftCorner<3,3>();
    t_ = T_.topRightCorner<3,1>();
  }

  SE3& operator=(const SE3& rhs);

  SE3 operator*(const SE3& rhs) const noexcept;

  V3  operator*(const V3& point) const noexcept;

  V4  operator*(const V4& point) const noexcept;
  
  void update(const V6 &dxi);

  void updateRhs(const V6 &dxi);

  void exp(const M4 &xi_hat);

  M4 log(void) const noexcept;

  V6 log_vee() const noexcept;

  V3 transform(const V3 & p) const noexcept;

  SE3 inverse(void) const noexcept;

  M6 adjoint() const noexcept;

  M4 T() const noexcept;

  M4 matrix() const noexcept;

  M4& ref2T();

  SO3 so3() const noexcept;

  M3 R() const noexcept;

  V3 translation() const noexcept;

  V3 t() const noexcept;

  Quat quaternion() const noexcept;

  double distance(const SE3 &rhs) const noexcept;

  void print(void) const noexcept;

  void print_lie(void) const noexcept;

public:
  M4 T_;
  M3 R_;
  V3 t_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/// only can be used for gravity vector.
class S2
{
public:
  V3 vec_ = V3(0, 0, 1);
  scalar length_ = 1;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  S2() = default;

  S2(scalar length, V3 = V3(0, 0, 1));
    
  S2(scalar length, scalar d1, scalar d2, scalar d3);

  /// Construct the local basis matrix on S² (tangent space basis).
  void S2_Bx(M3_2& Bx);

  /// Used to compute Jacobians under perturbations in the tangent space.
  void S2_Mx(M3_2 &Mx, const V2 &delta);

  void S2_Nx_yy(M2_3 &Nx);

  void S2_hat(M3 &res);
  
  void update (const V2& delta);

};

}

#endif