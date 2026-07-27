
#include "basic/Manifold.h"


namespace BASIC {

M3 A_matrix(const V3& v) {
  M3 res;
  double squaredNorm = v.squaredNorm();
  double norm = std::sqrt(squaredNorm);
  // c1 = (1-cos(t))/t^2, c2 = (t-sin(t))/t^3
  double c1, c2;
  if (norm < 1e-2) {
    // 小角稳定泰勒展开(同 SO3::Exp_m3)
    // c1 ≈ 1/2 - θ^2/24 + θ^4/720
    // c2 ≈ 1/6 - θ^2/120 + θ^4/5040
    const double t4 = squaredNorm * squaredNorm;
    c1 = 0.5 - squaredNorm / 24.0 + t4 / 720.0;
    c2 = 1.0 / 6.0 - squaredNorm / 120.0 + t4 / 5040.0;
  } else {
    c1 = (1 - std::cos(norm)) / squaredNorm;
    c2 = (1 - std::sin(norm) / norm) / squaredNorm;
  }
  const M3 vhat = SO3::hat(v);
  res = M3::Identity() + c1 * vhat + c2 * vhat * vhat;
  return res;
}


/**
 * SO3 and SE3 ref to https://github.com/hku-mars/BALM/blob/master/src/compare_test/SE3/SE3.hpp
 */


SO3::SO3(const M3 &R):R_(R){}

SO3::SO3(const V3 &w):R_(Eye3){
  this->exp(hat(w));
}

SO3::SO3(const SO3 &R):R_(R.R()){}

M3 SO3::hat(const V3 &v){
  M3 m;
  m << 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0;
  return m;
}

V3 SO3::vee(const M3 &w_hat){
  V3 w;
  w << -w_hat(1,2), w_hat(0,2), -w_hat(0,1);
  return w;
}

SO3 SO3::Exp(const V3 &ang) {
  scalar norm = ang.norm();
  M3 I = Eye3;
  if (norm > 1e-10) {
    V3 r_ang = ang / norm;
    M3 K;
    K = hat(r_ang);
    return SO3(I + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K);
  } else {
    return SO3(I);
  }
}


M3 SO3::Exp_m3(const V3 &ang_vel, const scalar &dt){
  static const scalar coefficient[] = {1.0/6.0, 1.0/24.0, 1.0/120.0, 1.0/720.0};

  V3 phi = ang_vel * dt;  
  scalar squaredNorm = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
  scalar norm = std::sqrt(squaredNorm);
  scalar A, B;
  if (squaredNorm >= 1e-6) {
      A = sin(norm) / norm;
      B = (1.0 - cos(norm)) / squaredNorm;
  } else {
      // 小角稳定泰勒展开到 θ^4
      // A ≈ 1 - θ^2/6 + θ^4/120
      // B ≈ 1/2 - θ^2/24 + θ^4/720
      const scalar t4 = squaredNorm * squaredNorm;
      A = 1.0 - (squaredNorm * coefficient[0]) + (t4 * coefficient[2]);
      B = 0.5 - (squaredNorm * coefficient[1]) + (t4 * coefficient[3]);
  }

  M3 K;
  K << 0.0, -phi[2], phi[1], phi[2], 0.0, -phi[0], -phi[1], phi[0], 0.0;
  return M3::Identity() + A * K + B * K * K;
}


SO3 SO3::Exp(const V3 &ang_vel, const scalar &dt) {
  scalar ang_vel_norm = ang_vel.norm();
  M3 I = Eye3;
  if (ang_vel_norm > 1e-12) {
    V3 r_axis = ang_vel / ang_vel_norm;
    M3 K;
    K = hat(r_axis);
    scalar r_ang = ang_vel_norm * dt;
    return SO3(I + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K);
  } else {
    return SO3(I);
  }
}

SO3& SO3::operator=(const SO3& rhs){
  if (this == &rhs)
      return *this;
  R_ = rhs.R();
  return *this;
}

SO3 SO3::operator*(const SO3& rhs) const noexcept{
  M3 res = R_ * rhs.R();
  return SO3(res);
}

SO3 SO3::operator*(scalar s) const noexcept{
  M3 res = R_ * s;
  return SO3(res);
}

V3 SO3::operator*(const V3& rhs) const noexcept{
  return R_ * rhs;
}

void SO3::update(const V3 &dw){
  SO3 dR(dw);
  R_ = dR.R() * R_;
}

void SO3::updateRhs(const V3 &dw){
  SO3 dR(dw);
  R_ = R_ * dR.R();
}

void SO3::exp(const M3 &w_hat){
  V3 w = vee(w_hat);
  double o = w.norm();
  if ( o < 1e-12){
      R_ << Eye3 + w_hat;
      return;
  }
  double c1 = std::sin(o)/o;
  double c2 = (1 - std::cos(o))/o/o;
  R_ << Eye3 + c1 * w_hat + c2 * w_hat *w_hat;
}

V4 SO3::coeffs() const noexcept{
  return Quat(R_).normalized().coeffs();
}


M3 SO3::log(double* ro) const noexcept
{
  const Eigen::Matrix3d R_double = R_.cast<double>();

  Eigen::Quaterniond q(R_double);
  q.normalize();

  if(q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }

  const Eigen::Vector3d vector_part(
      q.x(),
      q.y(),
      q.z()
  );

  const double sin_half_theta = vector_part.norm();

  Eigen::Vector3d rotation_vector;
  double theta = 0.0;

  if(sin_half_theta < 1e-12) {
    rotation_vector = 2.0 * vector_part;
    theta = rotation_vector.norm();
  }
  else {
    theta = 2.0 * std::atan2(
        sin_half_theta,
        q.w()
    );

    rotation_vector =
        theta / sin_half_theta
        * vector_part;
  }

  if(ro != nullptr) {
    *ro = theta;
  }

  return SO3::hat(
      rotation_vector.cast<scalar>()
  );
}


V3 SO3::log_vee() const noexcept{
  M3 w_hat = this->log();
  return vee(w_hat);
}

SO3 SO3::inverse(void) const noexcept{
  return SO3(R_.transpose());
}

M3 SO3::adjoint() const noexcept{
  // Ad_R = R: R * hat(w) * R^T = hat(R * w)
  return R_;
}

M3 SO3::R() const noexcept{
  return R_;
}

Quat SO3::quaternion() const noexcept{
  Quat res = Quat(R_);
  return res.normalized();
}

M3 SO3::matrix() const noexcept{
  return R_;
}

M3& SO3::ref2R(){
  return R_;
}

scalar SO3::distance(const SO3 &rhs) const noexcept{
  return (*this * rhs.inverse()).log_vee().norm();
}

void SO3::print(void) const noexcept{
  std::cout << R_ << std::endl;
}

void SO3::print_lie(void) const noexcept{
  V3 w =  this->log_vee();
  std::cout << w << std::endl;
}

scalar SO3::yaw() const noexcept{
  scalar sy = sqrt(R_(0, 0) * R_(0, 0) + R_(1, 0) * R_(1, 0));
  if(sy < 1e-6) 
    return 0;
  else
  return atan2(R_(1, 0), R_(0, 0));
}

////////////////////////////////////////////////////////////////////////
SE3::SE3(const M4 &T): T_(T) {
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}

SE3::SE3(const V6 &xi){
  this->exp(hat6(xi));
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}
  
SE3::SE3(const SE3 &T): T_(T.T()),R_(T.R()),t_(T.t()){}

SE3::SE3(const SO3 &R, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = R.R();
  T_.block<3,1>(0,3) = t;
  R_ = R.R();
  t_ = t;
}

SE3::SE3(const M3 &R, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = R;
  T_.block<3,1>(0,3) = t;
  R_ = R;
  t_ = t;
}

SE3::SE3(const Quat &q, const V3 &t):
T_(M4::Identity())
{
  T_.block<3,3>(0,0) = q.toRotationMatrix();
  T_.block<3,1>(0,3) = t;
  R_ = T_.topLeftCorner<3,3>();
  t_ = t;
}

SE3& SE3::operator=(const SE3& rhs){
  if (this == &rhs)
      return *this;
  T_ = rhs.T();
  R_ = rhs.R();
  t_ = rhs.t();
  return *this;
}

SE3 SE3::operator*(const SE3& rhs) const noexcept{
  M4 res = T_ * rhs.T();
  return SE3(res);
}

V3 SE3::operator*(const V3& point) const noexcept{
  return R_*point + t_;
}

V4 SE3::operator*(const V4& point) const noexcept{
  return T_ * point;
}

void SE3::update(const V6 &dxi){
  SE3 dT(dxi);
  T_ = dT.T() * T_;
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}

void SE3::updateRhs(const V6 &dxi){
  SE3 dT(dxi);
  T_ = T_ * dT.T();
  R_ = T_.topLeftCorner<3,3>();
  t_ = T_.topRightCorner<3,1>();
}

void SE3::exp(const M4 &xi_hat){
  V6 xi = vee6(xi_hat);
  V3 w = xi.head<3>();
  V3 v = xi.tail<3>();
  SO3 rotation(w);
  M3 w_hat = xi_hat.topLeftCorner<3,3>();

  M3 V = M3::Identity();
  double o = w.norm();
  if ( o > 1e-12){
    double c2 = (1 - std::cos(o))/o/o;
    double c3 = (o - std::sin(o))/o/o/o;
    V += c2*w_hat + c3*w_hat*w_hat;
  }
  V3 t = V * v;

  T_  << rotation.R(), t,
          0,0,0,1;
  R_ = rotation.R();
  t_ = t;
}

M4 SE3::log(void) const noexcept{
  SO3 rotation(this->R());
  double o;
  M3 w_hat = rotation.log(&o);
  M3 Vinv = M3::Identity();
  if (o > 1e-12)
  {
    // k1 = (1 - 0.5*o*sin(o)/(1-cos(o))) / o^2 suffers catastrophic
    // cancellation for small o (garbage below ~1e-3, 0/0 below ~2e-8 where
    // 1-cos(o) underflows to 0); use the series
    // k1 = 1/12 + o^2/720 + o^4/30240 + O(o^6) there.
    double k1;
    if (o < 1e-2) {
      const double o2 = o * o;
      k1 = 1.0 / 12.0 + o2 / 720.0 + o2 * o2 / 30240.0;
    } else {
      double c1 = std::sin(o);
      double c2 = (1 - std::cos(o))/o;
      k1 = 1/o/o*(1 - 0.5*c1/c2);
    }
    Vinv += -0.5*w_hat + k1* w_hat*w_hat;
  }
  V3 v = Vinv * T_.topRightCorner<3,1>();

  M4 xi_hat = M4::Zero();
  xi_hat << w_hat, v,
            0,0,0,0;
  return xi_hat;
}

V6 SE3::log_vee() const noexcept{
  M4 xi_hat = this->log();
  return vee6(xi_hat);
}

V3 SE3::transform(const V3 & p) const noexcept{
  return R_*p + t_;
}

SE3 SE3::inverse(void) const noexcept{
  M4 inv;
  M3 R = R_;
  R.transposeInPlace();
  inv << R, -R * this->t(),
          0,0,0,1;
  return SE3(inv);
}

M6 SE3::adjoint() const noexcept{
  M6 res(M6::Zero());
  M3 tx = SO3::hat( this->t() );
  res.topLeftCorner<3,3>() << R();
  res.bottomRightCorner<3,3>() << R();
  res.bottomLeftCorner<3,3>() << tx*R();
  return res;
}

M4 SE3::T() const noexcept{
  return T_;
}

M4 SE3::matrix() const noexcept{
  return T_;
}

M4& SE3::ref2T(){
  return T_;
}

SO3 SE3::so3() const noexcept{
  return SO3(R_);
}

M3 SE3::R() const noexcept{
  return R_;
}

V3 SE3::translation() const noexcept{
  return t_;
}

V3 SE3::t() const noexcept{
  return t_;
}

Quat SE3::quaternion() const noexcept{
  Quat res = Quat(R());
  return res.normalized();
}

double SE3::distance(const SE3 &rhs) const noexcept{
  return (*this * rhs.inverse()).log_vee().norm();
}

void SE3::print(void) const noexcept{
  std::cout << T_ << std::endl;
}

void SE3::print_lie(void) const noexcept{
  std::cout << this->log_vee() << std::endl;
}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

S2::S2(scalar length, V3 vec){
  length_ = length;
  vec_ = length_ * vec.normalized();
}

S2::S2(scalar length, scalar d1, scalar d2, scalar d3){
  V3 input = V3(d1, d2, d3);
  length_ = length;
  vec_ = length_ * input.normalized();
}


void S2::S2_Bx(M3_2& Bx) {
  V3 d = vec_.normalized();
  V3 v = (std::fabs(d.x()) < 0.9) ? V3(1, 0, 0) : V3(0, 1, 0);
  V3 b1 = (v - d * (d.dot(v))).normalized();
  V3 b2 = d.cross(b1);
  Bx.col(0) = b1;
  Bx.col(1) = b2;
}


void S2::S2_Mx(M3_2 &Mx, const V2 &delta){
  M3_2 Bx;
  S2_Bx(Bx);
  if (delta.norm() < tolerance<scalar>()) {
    Mx = - SO3::hat(vec_) * Bx;
  } else {
    V3 Bu = Bx * delta; 
    SO3 exp_delta = SO3::Exp(Bu);
    Mx = - exp_delta.R_ * SO3::hat(vec_) * A_matrix(Bu).transpose() * Bx;
  }
}


void S2::S2_Nx_yy(M2_3 &Nx) {
  M3_2 Bx;
  S2_Bx(Bx);
  Nx = 1 / length_ / length_ * Bx.transpose() * SO3::hat(vec_);
}


void S2::S2_hat(M3 &res) {
  M3 skew_vec;
  skew_vec << scalar(0), -vec_[2], vec_[1], vec_[2], scalar(0), -vec_[0], -vec_[1], vec_[0], scalar(0);
  res = skew_vec;
}


void S2::update (const V2& delta){
  M3_2 Bx;
  S2_Bx(Bx);
  V3 Bu = Bx * delta;
  vec_ = SO3::Exp(Bu).R_ * vec_;
}

}
