#include "types.hpp"

namespace rocos {

JntArray::JntArray() {}

JntArray::JntArray(unsigned int _size) : data(_size) { data.setZero(); }

JntArray::JntArray(const JntArray& arg) : data(arg.data) {}

JntArray& JntArray::operator=(const JntArray& arg) {
  data = arg.data;
  return *this;
}

JntArray::~JntArray() {}

void JntArray::resize(unsigned int newSize) {
  data.conservativeResizeLike(Eigen::VectorXd::Zero(newSize));
}

double JntArray::operator()(unsigned int i, unsigned int j) const {
  assert(j == 0);
  return data(i);
}

double& JntArray::operator()(unsigned int i, unsigned int j) {
  assert(j == 0);
  return data(i);
}

unsigned int JntArray::rows() const {
  return static_cast<unsigned int>(data.rows());
}

unsigned int JntArray::columns() const {
  return static_cast<unsigned int>(data.cols());
}

void Add(const JntArray& src1, const JntArray& src2, JntArray& dest) {
  dest.data = src1.data + src2.data;
}

void Subtract(const JntArray& src1, const JntArray& src2, JntArray& dest) {
  dest.data = src1.data - src2.data;
}

void Multiply(const JntArray& src, const double& factor, JntArray& dest) {
  dest.data = factor * src.data;
}

void Divide(const JntArray& src, const double& factor, JntArray& dest) {
  dest.data = src.data / factor;
}

void MultiplyJacobian(const Jacobian& jac, const JntArray& src, Twist& dest) {
  Eigen::Matrix<double, 6, 1> t = jac.data.lazyProduct(src.data);
  dest = Twist(Vector(t(0), t(1), t(2)), Vector(t(3), t(4), t(5)));
}

void SetToZero(JntArray& array) { array.data.setZero(); }

bool Equal(const JntArray& src1, const JntArray& src2, double eps) {
  if (src1.rows() != src2.rows()) return false;
  return (src1.data - src2.data).isZero(eps);
}

bool operator==(const JntArray& src1, const JntArray& src2) {
  return Equal(src1, src2);
}



Vector::Vector(const Vector& arg) {
  data[0] = arg.data[0];
  data[1] = arg.data[1];
  data[2] = arg.data[2];
}

Vector::Vector(double x, double y, double z) {
  data[0] = x;
  data[1] = y;
  data[2] = z;
}

Vector& Vector::operator=(const Vector& arg) {
  data[0] = arg.data[0];
  data[1] = arg.data[1];
  data[2] = arg.data[2];
  return *this;
}

Vector operator+(const Vector& lhs, const Vector& rhs) {
  Vector tmp;
  tmp.data[0] = lhs.data[0] + rhs.data[0];
  tmp.data[1] = lhs.data[1] + rhs.data[1];
  tmp.data[2] = lhs.data[2] + rhs.data[2];
  return tmp;
}

Vector operator-(const Vector& lhs, const Vector& rhs) {
  Vector tmp;
  tmp.data[0] = lhs.data[0] - rhs.data[0];
  tmp.data[1] = lhs.data[1] - rhs.data[1];
  tmp.data[2] = lhs.data[2] - rhs.data[2];
  return tmp;
}

double Vector::x() const { return data[0]; }
double Vector::y() const { return data[1]; }
double Vector::z() const { return data[2]; }

void Vector::x(double _x) { data[0] = _x; }
void Vector::y(double _y) { data[1] = _y; }
void Vector::z(double _z) { data[2] = _z; }

Vector operator*(const Vector& lhs, double rhs) {
  Vector tmp;
  tmp.data[0] = lhs.data[0] * rhs;
  tmp.data[1] = lhs.data[1] * rhs;
  tmp.data[2] = lhs.data[2] * rhs;
  return tmp;
}

Vector operator*(double lhs, const Vector& rhs) {
  Vector tmp;
  tmp.data[0] = lhs * rhs.data[0];
  tmp.data[1] = lhs * rhs.data[1];
  tmp.data[2] = lhs * rhs.data[2];
  return tmp;
}

Vector operator/(const Vector& lhs, double rhs) {
  Vector tmp;
  tmp.data[0] = lhs.data[0] / rhs;
  tmp.data[1] = lhs.data[1] / rhs;
  tmp.data[2] = lhs.data[2] / rhs;
  return tmp;
}

Vector operator*(const Vector& lhs, const Vector& rhs)
// Complexity : 6M+3A
{
  Vector tmp;
  tmp.data[0] = lhs.data[1] * rhs.data[2] - lhs.data[2] * rhs.data[1];
  tmp.data[1] = lhs.data[2] * rhs.data[0] - lhs.data[0] * rhs.data[2];
  tmp.data[2] = lhs.data[0] * rhs.data[1] - lhs.data[1] * rhs.data[0];
  return tmp;
}

Vector& Vector::operator+=(const Vector& arg)
// Complexity : 3A
{
  data[0] += arg.data[0];
  data[1] += arg.data[1];
  data[2] += arg.data[2];
  return *this;
}

Vector& Vector::operator-=(const Vector& arg)
// Complexity : 3A
{
  data[0] -= arg.data[0];
  data[1] -= arg.data[1];
  data[2] -= arg.data[2];
  return *this;
}

Vector Vector::Zero() { return Vector(0, 0, 0); }

double Vector::operator()(int index) const {
  assert((0 <= index) && (index <= 2));
  return data[index];
}

double& Vector::operator()(int index) {
  assert((0 <= index) && (index <= 2));
  return data[index];
}

/////////////FRAME/////////////////
Wrench Frame::operator*(const Wrench& arg) const
// Complexity : 24M+18A
{
  Wrench tmp;
  tmp.force = M * arg.force;
  tmp.torque = M * arg.torque + p * tmp.force;
  return tmp;
}

Wrench Frame::Inverse(const Wrench& arg) const {
  Wrench tmp;
  tmp.force = M.Inverse(arg.force);
  tmp.torque = M.Inverse(arg.torque - p * arg.force);
  return tmp;
}

/////////////ROTATION/////////////////
Wrench Rotation::Inverse(const Wrench& arg) const {
  return Wrench(Inverse(arg.force), Inverse(arg.torque));
}

Twist Rotation::Inverse(const Twist& arg) const {
  return Twist(Inverse(arg.vel), Inverse(arg.rot));
}

/////////////Wrench/////////////////
Wrench Wrench::Zero() { return Wrench(Vector::Zero(), Vector::Zero()); }

void Wrench::ReverseSign() {
  torque.ReverseSign();
  force.ReverseSign();
}

Wrench Wrench::RefPoint(const Vector& v_base_AB) const
// Changes the reference point of the Wrench.
// The vector v_base_AB is expressed in the same base as the twist
// The vector v_base_AB is a vector from the old point to
// the new point.
{
  return Wrench(this->force, this->torque + this->force * v_base_AB);
}

Wrench& Wrench::operator-=(const Wrench& arg) {
  torque -= arg.torque;
  force -= arg.force;
  return *this;
}

Wrench& Wrench::operator+=(const Wrench& arg) {
  torque += arg.torque;
  force += arg.force;
  return *this;
}

double& Wrench::operator()(int i) {
  // assert((0<=i)&&(i<6)); done by underlying routines
  if (i < 3)
    return force(i);
  else
    return torque(i - 3);
}

double Wrench::operator()(int i) const {
  // assert((0<=i)&&(i<6)); done by underlying routines
  if (i < 3)
    return force(i);
  else
    return torque(i - 3);
}

Wrench operator*(const Wrench& lhs, double rhs) {
  return Wrench(lhs.force * rhs, lhs.torque * rhs);
}

Wrench operator*(double lhs, const Wrench& rhs) {
  return Wrench(lhs * rhs.force, lhs * rhs.torque);
}

Wrench operator/(const Wrench& lhs, double rhs) {
  return Wrench(lhs.force / rhs, lhs.torque / rhs);
}

// addition of Wrench's
Wrench operator+(const Wrench& lhs, const Wrench& rhs) {
  return Wrench(lhs.force + rhs.force, lhs.torque + rhs.torque);
}

Wrench operator-(const Wrench& lhs, const Wrench& rhs) {
  return Wrench(lhs.force - rhs.force, lhs.torque - rhs.torque);
}

// unary -
Wrench operator-(const Wrench& arg) { return Wrench(-arg.force, -arg.torque); }

Twist Frame::operator*(const Twist& arg) const
// Complexity : 24M+18A
{
  Twist tmp;
  tmp.rot = M * arg.rot;
  tmp.vel = M * arg.vel + p * tmp.rot;
  return tmp;
}
Twist Frame::Inverse(const Twist& arg) const {
  Twist tmp;
  tmp.rot = M.Inverse(arg.rot);
  tmp.vel = M.Inverse(arg.vel - p * arg.rot);
  return tmp;
}

Twist Twist::Zero() { return Twist(Vector::Zero(), Vector::Zero()); }

void Twist::ReverseSign() {
  vel.ReverseSign();
  rot.ReverseSign();
}

Twist Twist::RefPoint(const Vector& v_base_AB) const
// Changes the reference point of the twist.
// The vector v_base_AB is expressed in the same base as the twist
// The vector v_base_AB is a vector from the old point to
// the new point.
// Complexity : 6M+6A
{
  return Twist(this->vel + this->rot * v_base_AB, this->rot);
}

Twist& Twist::operator-=(const Twist& arg) {
  vel -= arg.vel;
  rot -= arg.rot;
  return *this;
}

Twist& Twist::operator+=(const Twist& arg) {
  vel += arg.vel;
  rot += arg.rot;
  return *this;
}

double& Twist::operator()(int i) {
  // assert((0<=i)&&(i<6)); done by underlying routines
  if (i < 3)
    return vel(i);
  else
    return rot(i - 3);
}

double Twist::operator()(int i) const {
  // assert((0<=i)&&(i<6)); done by underlying routines
  if (i < 3)
    return vel(i);
  else
    return rot(i - 3);
}

Twist operator*(const Twist& lhs, double rhs) {
  return Twist(lhs.vel * rhs, lhs.rot * rhs);
}

Twist operator*(double lhs, const Twist& rhs) {
  return Twist(lhs * rhs.vel, lhs * rhs.rot);
}

Twist operator/(const Twist& lhs, double rhs) {
  return Twist(lhs.vel / rhs, lhs.rot / rhs);
}

// addition of Twist's
Twist operator+(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.vel + rhs.vel, lhs.rot + rhs.rot);
}

Twist operator-(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.vel - rhs.vel, lhs.rot - rhs.rot);
}

// unary -
Twist operator-(const Twist& arg) { return Twist(-arg.vel, -arg.rot); }

// Spatial products for twists
Twist operator*(const Twist& lhs, const Twist& rhs) {
  return Twist(lhs.rot * rhs.vel + lhs.vel * rhs.rot, lhs.rot * rhs.rot);
}
Wrench operator*(const Twist& lhs, const Wrench& rhs) {
  return Wrench(lhs.rot * rhs.force,
                lhs.rot * rhs.torque + lhs.vel * rhs.force);
}

Frame::Frame(const Rotation& R) {
  M = R;
  p = Vector::Zero();
}

Frame::Frame(const Vector& V) {
  M = Rotation::Identity();
  p = V;
}

Frame::Frame(const Rotation& R, const Vector& V) {
  M = R;
  p = V;
}

Frame operator*(const Frame& lhs, const Frame& rhs)
// Complexity : 36M+36A
{
  return Frame(lhs.M * rhs.M, lhs.M * rhs.p + lhs.p);
}

Vector Frame::operator*(const Vector& arg) const { return M * arg + p; }

Vector Frame::Inverse(const Vector& arg) const { return M.Inverse(arg - p); }

Frame Frame::Inverse() const { return Frame(M.Inverse(), -M.Inverse(p)); }

Frame& Frame::operator=(const Frame& arg) {
  M = arg.M;
  p = arg.p;
  return *this;
}

Frame::Frame(const Frame& arg) : p(arg.p), M(arg.M) {}

void Vector::ReverseSign() {
  data[0] = -data[0];
  data[1] = -data[1];
  data[2] = -data[2];
}

Vector operator-(const Vector& arg) {
  Vector tmp;
  tmp.data[0] = -arg.data[0];
  tmp.data[1] = -arg.data[1];
  tmp.data[2] = -arg.data[2];
  return tmp;
}

double& Rotation::operator()(int i, int j) {
  assert((0 <= i) && (i <= 2) && (0 <= j) && (j <= 2));
  return data[i * 3 + j];
}

double Rotation::operator()(int i, int j) const {
  assert((0 <= i) && (i <= 2) && (0 <= j) && (j <= 2));
  return data[i * 3 + j];
}

Rotation::Rotation(double Xx, double Yx, double Zx, double Xy, double Yy,
                   double Zy, double Xz, double Yz, double Zz) {
  data[0] = Xx;
  data[1] = Yx;
  data[2] = Zx;
  data[3] = Xy;
  data[4] = Yy;
  data[5] = Zy;
  data[6] = Xz;
  data[7] = Yz;
  data[8] = Zz;
}

Rotation::Rotation(const Vector& x, const Vector& y, const Vector& z) {
  data[0] = x.data[0];
  data[3] = x.data[1];
  data[6] = x.data[2];
  data[1] = y.data[0];
  data[4] = y.data[1];
  data[7] = y.data[2];
  data[2] = z.data[0];
  data[5] = z.data[1];
  data[8] = z.data[2];
}

Rotation::Rotation(const Rotation& arg) {
  int count = 9;
  while (count--) data[count] = arg.data[count];
}

Rotation& Rotation::operator=(const Rotation& arg) {
  int count = 9;
  while (count--) data[count] = arg.data[count];
  return *this;
}

Vector Rotation::operator*(const Vector& v) const {
  // Complexity : 9M+6A
  return Vector(
      data[0] * v.data[0] + data[1] * v.data[1] + data[2] * v.data[2],
      data[3] * v.data[0] + data[4] * v.data[1] + data[5] * v.data[2],
      data[6] * v.data[0] + data[7] * v.data[1] + data[8] * v.data[2]);
}

Twist Rotation::operator*(const Twist& arg) const
// Transformation of the base to which the twist is expressed.
// look at Frame*Twist for a transformation that also transforms
// the velocity reference point.
// Complexity : 18M+12A
{
  return Twist((*this) * arg.vel, (*this) * arg.rot);
}

Wrench Rotation::operator*(const Wrench& arg) const
// Transformation of the base to which the wrench is expressed.
// look at Frame*Twist for a transformation that also transforms
// the force reference point.
{
  return Wrench((*this) * arg.force, (*this) * arg.torque);
}

Rotation Rotation::Identity() { return Rotation(1, 0, 0, 0, 1, 0, 0, 0, 1); }
// *this = *this * ROT(X,angle)
void Rotation::DoRotX(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  double x1, x2, x3;
  x1 = cs * (*this)(0, 1) + sn * (*this)(0, 2);
  x2 = cs * (*this)(1, 1) + sn * (*this)(1, 2);
  x3 = cs * (*this)(2, 1) + sn * (*this)(2, 2);
  (*this)(0, 2) = -sn * (*this)(0, 1) + cs * (*this)(0, 2);
  (*this)(1, 2) = -sn * (*this)(1, 1) + cs * (*this)(1, 2);
  (*this)(2, 2) = -sn * (*this)(2, 1) + cs * (*this)(2, 2);
  (*this)(0, 1) = x1;
  (*this)(1, 1) = x2;
  (*this)(2, 1) = x3;
}

void Rotation::DoRotY(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  double x1, x2, x3;
  x1 = cs * (*this)(0, 0) - sn * (*this)(0, 2);
  x2 = cs * (*this)(1, 0) - sn * (*this)(1, 2);
  x3 = cs * (*this)(2, 0) - sn * (*this)(2, 2);
  (*this)(0, 2) = sn * (*this)(0, 0) + cs * (*this)(0, 2);
  (*this)(1, 2) = sn * (*this)(1, 0) + cs * (*this)(1, 2);
  (*this)(2, 2) = sn * (*this)(2, 0) + cs * (*this)(2, 2);
  (*this)(0, 0) = x1;
  (*this)(1, 0) = x2;
  (*this)(2, 0) = x3;
}

void Rotation::DoRotZ(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  double x1, x2, x3;
  x1 = cs * (*this)(0, 0) + sn * (*this)(0, 1);
  x2 = cs * (*this)(1, 0) + sn * (*this)(1, 1);
  x3 = cs * (*this)(2, 0) + sn * (*this)(2, 1);
  (*this)(0, 1) = -sn * (*this)(0, 0) + cs * (*this)(0, 1);
  (*this)(1, 1) = -sn * (*this)(1, 0) + cs * (*this)(1, 1);
  (*this)(2, 1) = -sn * (*this)(2, 0) + cs * (*this)(2, 1);
  (*this)(0, 0) = x1;
  (*this)(1, 0) = x2;
  (*this)(2, 0) = x3;
}

Rotation Rotation::RotX(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  return Rotation(1, 0, 0, 0, cs, -sn, 0, sn, cs);
}
Rotation Rotation::RotY(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  return Rotation(cs, 0, sn, 0, 1, 0, -sn, 0, cs);
}
Rotation Rotation::RotZ(double angle) {
  double cs = cos(angle);
  double sn = sin(angle);
  return Rotation(cs, -sn, 0, sn, cs, 0, 0, 0, 1);
}

void Frame::Integrate(const Twist& t_this, double samplefrequency) {
  double n = t_this.rot.Norm() / samplefrequency;
  if (n < EPSILON) {
    p += M * (t_this.vel / samplefrequency);
  } else {
    (*this) = (*this) *
              Frame(Rotation::Rot(t_this.rot, n), t_this.vel / samplefrequency);
  }
}

Rotation Rotation::Inverse() const {
  Rotation tmp(*this);
  tmp.SetInverse();
  return tmp;
}

Vector Rotation::Inverse(const Vector& v) const {
  return Vector(
      data[0] * v.data[0] + data[3] * v.data[1] + data[6] * v.data[2],
      data[1] * v.data[0] + data[4] * v.data[1] + data[7] * v.data[2],
      data[2] * v.data[0] + data[5] * v.data[1] + data[8] * v.data[2]);
}

void Rotation::SetInverse() {
  double tmp;
  tmp = data[1];
  data[1] = data[3];
  data[3] = tmp;
  tmp = data[2];
  data[2] = data[6];
  data[6] = tmp;
  tmp = data[5];
  data[5] = data[7];
  data[7] = tmp;
}

double Frame::operator()(int i, int j) {
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3) {
    if (j == 3)
      return 1.0;
    else
      return 0.0;
  } else {
    if (j == 3)
      return p(i);
    else
      return M(i, j);
  }
}

double Frame::operator()(int i, int j) const {
  assert((0 <= i) && (i <= 3) && (0 <= j) && (j <= 3));
  if (i == 3) {
    if (j == 3)
      return 1;
    else
      return 0;
  } else {
    if (j == 3)
      return p(i);
    else
      return M(i, j);
  }
}

Frame Frame::Identity() { return Frame(Rotation::Identity(), Vector::Zero()); }

// Scalar products.

double dot(const Vector& lhs, const Vector& rhs) {
  return rhs(0) * lhs(0) + rhs(1) * lhs(1) + rhs(2) * lhs(2);
}

double dot(const Twist& lhs, const Wrench& rhs) {
  return dot(lhs.vel, rhs.force) + dot(lhs.rot, rhs.torque);
}

double dot(const Wrench& rhs, const Twist& lhs) {
  return dot(lhs.vel, rhs.force) + dot(lhs.rot, rhs.torque);
}

// Equality operators

bool Equal(const Vector& a, const Vector& b, double eps) {
  return (Equal(a.data[0], b.data[0], eps) &&
          Equal(a.data[1], b.data[1], eps) && Equal(a.data[2], b.data[2], eps));
}

bool Equal(const Frame& a, const Frame& b, double eps) {
  return (Equal(a.p, b.p, eps) && Equal(a.M, b.M, eps));
}

bool Equal(const Wrench& a, const Wrench& b, double eps) {
  return (Equal(a.force, b.force, eps) && Equal(a.torque, b.torque, eps));
}

bool Equal(const Twist& a, const Twist& b, double eps) {
  return (Equal(a.rot, b.rot, eps) && Equal(a.vel, b.vel, eps));
}

void SetToZero(Vector& v) { v = Vector::Zero(); }
void SetToZero(Twist& v) {
  SetToZero(v.rot);
  SetToZero(v.vel);
}
void SetToZero(Wrench& v) {
  SetToZero(v.force);
  SetToZero(v.torque);
}

////////////////////////////////////////////////////////////////
// The following defines the operations
//   diff
//   addDelta
//   random
//   posrandom
// on all the types defined in this library.
// (mostly for uniform integration, differentiation and testing).
// Defined as functions because double is not a class and a method
// would brake uniformity when defined for a double.
////////////////////////////////////////////////////////////////

/**
 * axis_a_b is a rotation vector, its norm is a rotation angle
 * axis_a_b rotates the a frame towards the b frame.
 * This routine returns the rotation matrix R_a_b
 */
Rotation Rot(const Vector& axis_a_b) {
  // The formula is
  // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
  // can be found by multiplying it with an arbitrary vector p
  // and noting that this vector is rotated.
  Vector rotvec = axis_a_b;
  double angle = rotvec.Normalize(1E-10);
  double ct = ::cos(angle);
  double st = ::sin(angle);
  double vt = 1 - ct;
  return Rotation(ct + vt * rotvec(0) * rotvec(0),
                  -rotvec(2) * st + vt * rotvec(0) * rotvec(1),
                  rotvec(1) * st + vt * rotvec(0) * rotvec(2),
                  rotvec(2) * st + vt * rotvec(1) * rotvec(0),
                  ct + vt * rotvec(1) * rotvec(1),
                  -rotvec(0) * st + vt * rotvec(1) * rotvec(2),
                  -rotvec(1) * st + vt * rotvec(2) * rotvec(0),
                  rotvec(0) * st + vt * rotvec(2) * rotvec(1),
                  ct + vt * rotvec(2) * rotvec(2));
}
Vector diff(const Vector& a, const Vector& b, double dt) {
  return (b - a) / dt;
}

Vector diff(const Rotation& R_a_b1, const Rotation& R_a_b2, double dt) {
  Rotation R_b1_b2(R_a_b1.Inverse() * R_a_b2);
  return R_a_b1 * R_b1_b2.GetRot() / dt;
}

Twist diff(const Frame& F_a_b1, const Frame& F_a_b2, double dt) {
  return Twist(diff(F_a_b1.p, F_a_b2.p, dt), diff(F_a_b1.M, F_a_b2.M, dt));
}
Twist diff(const Twist& a, const Twist& b, double dt) {
  return Twist(diff(a.vel, b.vel, dt), diff(a.rot, b.rot, dt));
}

Wrench diff(const Wrench& a, const Wrench& b, double dt) {
  return Wrench(diff(a.force, b.force, dt), diff(a.torque, b.torque, dt));
}

Vector addDelta(const Vector& a, const Vector& da, double dt) {
  return a + da * dt;
}

Rotation addDelta(const Rotation& a, const Vector& da, double dt) {
  return Rot(da * dt) * a;
}
Frame addDelta(const Frame& a, const Twist& da, double dt) {
  return Frame(addDelta(a.M, da.rot, dt), addDelta(a.p, da.vel, dt));
}
Twist addDelta(const Twist& a, const Twist& da, double dt) {
  return Twist(addDelta(a.vel, da.vel, dt), addDelta(a.rot, da.rot, dt));
}
Wrench addDelta(const Wrench& a, const Wrench& da, double dt) {
  return Wrench(addDelta(a.force, da.force, dt),
                addDelta(a.torque, da.torque, dt));
}

/**
 * \brief addDelta operator for displacement rotational velocity.
 *
 * The Vector arguments here represent a displacement rotational velocity.  i.e.
a rotation
 * around a fixed axis for a certain angle.  For this representation you cannot
use diff() but
 * have to use diff_displ().
 *
 * \param a  : displacement rotational velocity
 * \param da : rotational velocity
 * \return   displacement rotational velocity
 *
 * \warning do not confuse displacement rotational velocities and velocities
 * \warning do not confuse displacement twist and twist.
 *
inline Vector addDelta_displ(const Vector& a,const Vector&da,double dt) {
    return getRot(addDelta(Rot(a),da,dt));
}*/

/**
 * \brief addDelta operator for displacement twist.
 *
 * The Vector arguments here represent a displacement rotational velocity.  i.e.
a rotation
 * around a fixed axis for a certain angle.  For this representation you cannot
use diff() but
 * have to use diff_displ().
 *
 * \param a  : displacement twist
 * \param da : twist
 * \return   displacement twist
 *
 * \warning do not confuse displacement rotational velocities and velocities
 * \warning do not confuse displacement twist and twist.
 *
inline Twist addDelta_displ(const Twist& a,const Twist&da,double dt) {
    return Twist(addDelta(a.vel,da.vel,dt),addDelta_displ(a.rot,da.rot,dt));
}*/

void random(Vector& a) {
  random(a[0]);
  random(a[1]);
  random(a[2]);
}
void random(Twist& a) {
  random(a.rot);
  random(a.vel);
}
void random(Wrench& a) {
  random(a.torque);
  random(a.force);
}

void random(Rotation& R) {
  double alfa;
  double beta;
  double gamma;
  random(alfa);
  random(beta);
  random(gamma);
  R = Rotation::EulerZYX(alfa, beta, gamma);
}

void random(Frame& F) {
  random(F.M);
  random(F.p);
}

void posrandom(Vector& a) {
  posrandom(a[0]);
  posrandom(a[1]);
  posrandom(a[2]);
}

void posrandom(Twist& a) {
  posrandom(a.rot);
  posrandom(a.vel);
}

void posrandom(Wrench& a) {
  posrandom(a.torque);
  posrandom(a.force);
}

void posrandom(Rotation& R) {
  double alfa;
  double beta;
  double gamma;
  posrandom(alfa);
  posrandom(beta);
  posrandom(gamma);
  R = Rotation::EulerZYX(alfa, beta, gamma);
}

void posrandom(Frame& F) {
  random(F.M);
  random(F.p);
}

bool operator==(const Frame& a, const Frame& b) {
#ifdef KDL_USE_EQUAL
  return Equal(a, b);
#else
  return (a.p == b.p && a.M == b.M);
#endif
}

bool operator!=(const Frame& a, const Frame& b) {
  return !operator==(a, b);
}

bool operator==(const Vector& a, const Vector& b) {
#ifdef KDL_USE_EQUAL
  return Equal(a, b);
#else
  return (a.data[0] == b.data[0] && a.data[1] == b.data[1] &&
          a.data[2] == b.data[2]);
#endif
}

bool operator!=(const Vector& a, const Vector& b) {
  return !operator==(a, b);
}

bool operator==(const Twist& a, const Twist& b) {
#ifdef KDL_USE_EQUAL
  return Equal(a, b);
#else
  return (a.rot == b.rot && a.vel == b.vel);
#endif
}

bool operator!=(const Twist& a, const Twist& b) {
  return !operator==(a, b);
}

bool operator==(const Wrench& a, const Wrench& b) {
#ifdef KDL_USE_EQUAL
  return Equal(a, b);
#else
  return (a.force == b.force && a.torque == b.torque);
#endif
}

bool operator!=(const Wrench& a, const Wrench& b) {
  return !operator==(a, b);
}
bool operator!=(const Rotation& a, const Rotation& b) {
  return !operator==(a, b);
}

void Frame::Make4x4(double* d) {
  int i;
  int j;
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) d[i * 4 + j] = M(i, j);
    d[i * 4 + 3] = p(i);
  }
  for (j = 0; j < 3; j++) d[12 + j] = 0.;
  d[15] = 1;
}

Frame Frame::DH_Craig1989(double a, double alpha, double d, double theta)
// returns Modified Denavit-Hartenberg parameters (According to Craig)
{
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);
  return Frame(
      Rotation(ct, -st, 0, st * ca, ct * ca, -sa, st * sa, ct * sa, ca),
      Vector(a, -sa * d, ca * d));
}

Frame Frame::DH(double a, double alpha, double d, double theta)
// returns Denavit-Hartenberg parameters (Non-Modified DH)
{
  double ct, st, ca, sa;
  ct = cos(theta);
  st = sin(theta);
  sa = sin(alpha);
  ca = cos(alpha);
  return Frame(
      Rotation(ct, -st * ca, st * sa, st, ct * ca, -ct * sa, 0, sa, ca),
      Vector(a * ct, a * st, d));
}

// do some effort not to lose precision
double Vector::Norm(double eps) const {
  double tmp1;
  double tmp2;
  tmp1 = fabs(data[0]);
  tmp2 = fabs(data[1]);
  if (tmp1 >= tmp2) {
    tmp2 = fabs(data[2]);
    if (tmp1 >= tmp2) {
      if (tmp1 < eps) {
        // only to everything exactly zero case, all other are handled correctly
        return 0;
      }
      return tmp1 * sqrt(1 + sqr(data[1] / data[0]) + sqr(data[2] / data[0]));
    } else {
      return tmp2 * sqrt(1 + sqr(data[0] / data[2]) + sqr(data[1] / data[2]));
    }
  } else {
    tmp1 = fabs(data[2]);
    if (tmp2 > tmp1) {
      return tmp2 * sqrt(1 + sqr(data[0] / data[1]) + sqr(data[2] / data[1]));
    } else {
      return tmp1 * sqrt(1 + sqr(data[0] / data[2]) + sqr(data[1] / data[2]));
    }
  }
}

// makes v a unitvector and returns the norm of v.
// if v is smaller than eps, Vector(1,0,0) is returned with norm 0.
// if this is not good, check the return value of this method.
double Vector::Normalize(double eps) {
  double v = this->Norm();
  if (v < eps) {
    *this = Vector(1, 0, 0);
    return 0;
  } else {
    *this = (*this) / v;
    return v;
  }
}

bool Equal(const Rotation& a, const Rotation& b, double eps) {
  return (
      Equal(a.data[0], b.data[0], eps) && Equal(a.data[1], b.data[1], eps) &&
      Equal(a.data[2], b.data[2], eps) && Equal(a.data[3], b.data[3], eps) &&
      Equal(a.data[4], b.data[4], eps) && Equal(a.data[5], b.data[5], eps) &&
      Equal(a.data[6], b.data[6], eps) && Equal(a.data[7], b.data[7], eps) &&
      Equal(a.data[8], b.data[8], eps));
}

Rotation operator*(const Rotation& lhs, const Rotation& rhs)
// Complexity : 27M+27A
{
  return Rotation(lhs.data[0] * rhs.data[0] + lhs.data[1] * rhs.data[3] +
                      lhs.data[2] * rhs.data[6],
                  lhs.data[0] * rhs.data[1] + lhs.data[1] * rhs.data[4] +
                      lhs.data[2] * rhs.data[7],
                  lhs.data[0] * rhs.data[2] + lhs.data[1] * rhs.data[5] +
                      lhs.data[2] * rhs.data[8],
                  lhs.data[3] * rhs.data[0] + lhs.data[4] * rhs.data[3] +
                      lhs.data[5] * rhs.data[6],
                  lhs.data[3] * rhs.data[1] + lhs.data[4] * rhs.data[4] +
                      lhs.data[5] * rhs.data[7],
                  lhs.data[3] * rhs.data[2] + lhs.data[4] * rhs.data[5] +
                      lhs.data[5] * rhs.data[8],
                  lhs.data[6] * rhs.data[0] + lhs.data[7] * rhs.data[3] +
                      lhs.data[8] * rhs.data[6],
                  lhs.data[6] * rhs.data[1] + lhs.data[7] * rhs.data[4] +
                      lhs.data[8] * rhs.data[7],
                  lhs.data[6] * rhs.data[2] + lhs.data[7] * rhs.data[5] +
                      lhs.data[8] * rhs.data[8]);
}

Rotation Rotation::Quaternion(double x, double y, double z, double w) {
  double x2, y2, z2, w2;
  x2 = x * x;
  y2 = y * y;
  z2 = z * z;
  w2 = w * w;
  return Rotation(
      w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y,
      2 * x * y + 2 * w * z, w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x,
      2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, w2 - x2 - y2 + z2);
}

/* From the following sources:
   http://web.archive.org/web/20041029003853/http:/www.j3d.org/matrix_faq/matrfaq_latest.html
   http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
   RobOOP::quaternion.cpp
*/
void Rotation::GetQuaternion(double& x, double& y, double& z, double& w) const {
  double trace = (*this)(0, 0) + (*this)(1, 1) + (*this)(2, 2);
  double epsilon = 1E-12;
  if (trace > epsilon) {
    double s = 0.5 / sqrt(trace + 1.0);
    w = 0.25 / s;
    x = ((*this)(2, 1) - (*this)(1, 2)) * s;
    y = ((*this)(0, 2) - (*this)(2, 0)) * s;
    z = ((*this)(1, 0) - (*this)(0, 1)) * s;
  } else {
    if ((*this)(0, 0) > (*this)(1, 1) && (*this)(0, 0) > (*this)(2, 2)) {
      double s =
          2.0 * sqrt(1.0 + (*this)(0, 0) - (*this)(1, 1) - (*this)(2, 2));
      w = ((*this)(2, 1) - (*this)(1, 2)) / s;
      x = 0.25 * s;
      y = ((*this)(0, 1) + (*this)(1, 0)) / s;
      z = ((*this)(0, 2) + (*this)(2, 0)) / s;
    } else if ((*this)(1, 1) > (*this)(2, 2)) {
      double s =
          2.0 * sqrt(1.0 + (*this)(1, 1) - (*this)(0, 0) - (*this)(2, 2));
      w = ((*this)(0, 2) - (*this)(2, 0)) / s;
      x = ((*this)(0, 1) + (*this)(1, 0)) / s;
      y = 0.25 * s;
      z = ((*this)(1, 2) + (*this)(2, 1)) / s;
    } else {
      double s =
          2.0 * sqrt(1.0 + (*this)(2, 2) - (*this)(0, 0) - (*this)(1, 1));
      w = ((*this)(1, 0) - (*this)(0, 1)) / s;
      x = ((*this)(0, 2) + (*this)(2, 0)) / s;
      y = ((*this)(1, 2) + (*this)(2, 1)) / s;
      z = 0.25 * s;
    }
  }
}

Rotation Rotation::RPY(double roll, double pitch, double yaw) {
  double ca1, cb1, cc1, sa1, sb1, sc1;
  ca1 = cos(yaw);
  sa1 = sin(yaw);
  cb1 = cos(pitch);
  sb1 = sin(pitch);
  cc1 = cos(roll);
  sc1 = sin(roll);
  return Rotation(ca1 * cb1, ca1 * sb1 * sc1 - sa1 * cc1,
                  ca1 * sb1 * cc1 + sa1 * sc1, sa1 * cb1,
                  sa1 * sb1 * sc1 + ca1 * cc1, sa1 * sb1 * cc1 - ca1 * sc1,
                  -sb1, cb1 * sc1, cb1 * cc1);
}

// Gives back a rotation matrix specified with RPY convention
void Rotation::GetRPY(double& roll, double& pitch, double& yaw) const {
  double epsilon = 1E-12;
  pitch = atan2(-data[6], sqrt(sqr(data[0]) + sqr(data[3])));
  if (fabs(pitch) > (PI_2 - epsilon)) {
    yaw = atan2(-data[1], data[4]);
    roll = 0.0;
  } else {
    roll = atan2(data[7], data[8]);
    yaw = atan2(data[3], data[0]);
  }
}

Rotation Rotation::EulerZYZ(double Alfa, double Beta, double Gamma) {
  double sa, ca, sb, cb, sg, cg;
  sa = sin(Alfa);
  ca = cos(Alfa);
  sb = sin(Beta);
  cb = cos(Beta);
  sg = sin(Gamma);
  cg = cos(Gamma);
  return Rotation(ca * cb * cg - sa * sg, -ca * cb * sg - sa * cg, ca * sb,
                  sa * cb * cg + ca * sg, -sa * cb * sg + ca * cg, sa * sb,
                  -sb * cg, sb * sg, cb);
}

void Rotation::GetEulerZYZ(double& alpha, double& beta, double& gamma) const {
  double epsilon = 1E-12;
  if (fabs(data[8]) > 1 - epsilon) {
    gamma = 0.0;
    if (data[8] > 0) {
      beta = 0.0;
      alpha = atan2(data[3], data[0]);
    } else {
      beta = PI;
      alpha = atan2(-data[3], -data[0]);
    }
  } else {
    alpha = atan2(data[5], data[2]);
    beta = atan2(sqrt(sqr(data[6]) + sqr(data[7])), data[8]);
    gamma = atan2(data[7], -data[6]);
  }
}

Rotation Rotation::Rot(const Vector& rotaxis, double angle) {
  // The formula is
  // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
  // can be found by multiplying it with an arbitrary vector p
  // and noting that this vector is rotated.
  Vector rotvec = rotaxis;
  rotvec.Normalize();
  return Rotation::Rot2(rotvec, angle);
}

Rotation Rotation::Rot2(const Vector& rotvec, double angle) {
  // rotvec should be normalized !
  // The formula is
  // V.(V.tr) + st*[V x] + ct*(I-V.(V.tr))
  // can be found by multiplying it with an arbitrary vector p
  // and noting that this vector is rotated.
  double ct = cos(angle);
  double st = sin(angle);
  double vt = 1 - ct;
  double m_vt_0 = vt * rotvec(0);
  double m_vt_1 = vt * rotvec(1);
  double m_vt_2 = vt * rotvec(2);
  double m_st_0 = rotvec(0) * st;
  double m_st_1 = rotvec(1) * st;
  double m_st_2 = rotvec(2) * st;
  double m_vt_0_1 = m_vt_0 * rotvec(1);
  double m_vt_0_2 = m_vt_0 * rotvec(2);
  double m_vt_1_2 = m_vt_1 * rotvec(2);
  return Rotation(ct + m_vt_0 * rotvec(0), -m_st_2 + m_vt_0_1,
                  m_st_1 + m_vt_0_2, m_st_2 + m_vt_0_1, ct + m_vt_1 * rotvec(1),
                  -m_st_0 + m_vt_1_2, -m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2,
                  ct + m_vt_2 * rotvec(2));
}

Vector Rotation::GetRot() const
// Returns a vector with the direction of the equiv. axis
// and its norm is angle
{
  Vector axis;
  double angle;
  angle = Rotation::GetRotAngle(axis, EPSILON);
  return axis * angle;
}

/** Returns the rotation angle around the equiv. axis
 * @param axis the rotation axis is returned in this variable
 * @param eps :  in the case of angle == 0 : rot axis is undefined and chosen
 *                                         to be the Z-axis
 *               in the case of angle == PI : 2 solutions, positive Z-component
 *                                            of the axis is chosen.
 * @result returns the rotation angle (between [0..PI] )
 * /todo :
 *   Check corresponding routines in rframes and rrframes
 */
double Rotation::GetRotAngle(Vector& axis, double eps) const {
  double angle, x, y, z;  // variables for result
  double epsilon = eps;   // margin to allow for rounding errors
  double epsilon2 =
      eps * 10;  // margin to distinguish between 0 and 180 degrees

  // optional check that input is pure rotation, 'isRotationMatrix' is defined
  // at: http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/

  if ((std::abs(data[1] - data[3]) < epsilon) &&
      (std::abs(data[2] - data[6]) < epsilon) &&
      (std::abs(data[5] - data[7]) < epsilon)) {
    // singularity found
    // first check for identity matrix which must have +1 for all terms
    //  in leading diagonal and zero in other terms
    if ((std::abs(data[1] + data[3]) < epsilon2) &&
        (std::abs(data[2] + data[6]) < epsilon2) &&
        (std::abs(data[5] + data[7]) < epsilon2) &&
        (std::abs(data[0] + data[4] + data[8] - 3) < epsilon2)) {
      // this singularity is identity matrix so angle = 0, axis is arbitrary
      // Choose 0, 0, 1 to pass orocos tests
      axis = Vector(0, 0, 1);
      angle = 0.0;
      return angle;
    }

    // otherwise this singularity is angle = 180
    angle = PI;
    double xx = (data[0] + 1) / 2;
    double yy = (data[4] + 1) / 2;
    double zz = (data[8] + 1) / 2;
    double xy = (data[1] + data[3]) / 4;
    double xz = (data[2] + data[6]) / 4;
    double yz = (data[5] + data[7]) / 4;

    if ((xx > yy) && (xx > zz)) {
      // data[0] is the largest diagonal term
      x = sqrt(xx);
      y = xy / x;
      z = xz / x;
    } else if (yy > zz) {
      // data[4] is the largest diagonal term
      y = sqrt(yy);
      x = xy / y;
      z = yz / y;
    } else {
      // data[8] is the largest diagonal term so base result on this
      z = sqrt(zz);
      x = xz / z;
      y = yz / z;
    }
    axis = Vector(x, y, z);
    return angle;  // return 180 deg rotation
  }

  double f = (data[0] + data[4] + data[8] - 1) / 2;

  x = (data[7] - data[5]);
  y = (data[2] - data[6]);
  z = (data[3] - data[1]);
  axis = Vector(x, y, z);
  angle = atan2(axis.Norm() / 2, f);
  axis.Normalize();
  return angle;
}

bool operator==(const Rotation& a, const Rotation& b) { return Equal(a, b); }

}  // namespace rocos