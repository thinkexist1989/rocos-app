// Copyright 2026, Yang Luo"
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// @Author
// Yang Luo, PHD
// Shenyang Institute of Automation, Chinese Academy of Sciences.
// email: luoyang@sia.cn
#pragma once

#include "utilities/utility.hpp"

#include <Eigen/Core>

namespace rocos {

class JntArray;
class Vector;
class Rotation;
class Frame;
class Wrench;
class Twist;
class Jacobian;

class JntArray {
 public:
  Eigen::VectorXd data;

  /** Construct with _no_ data array
   * @post NULL == data
   * @post 0 == rows()
   * @warning use of an object constructed like this, without
   * a resize() first, may result in program exit! See class
   * documentation.
   */
  JntArray();
  /**
   * Constructor of the joint array
   *
   * @param size size of the array, this cannot be changed
   * afterwards.
   * @pre 0 < size
   * @post NULL != data
   * @post 0 < rows()
   * @post all elements in data have 0 value
   */
  explicit JntArray(unsigned int size);
  /** Copy constructor
   * @note Will correctly copy an empty object
   */
  JntArray(const JntArray& other);

  ~JntArray();
  /** Resize the array
   * @warning This causes a dynamic allocation (and potentially
   * also a dynamic deallocation). This _will_ negatively affect
   * real-time performance!
   *
   * @post newSize == rows()
   * @post NULL != data
   * @post all elements in data have 0 value
   */
  void resize(unsigned int newSize);

  JntArray& operator=(const JntArray& other);
  /**
   * get_item operator for the joint array, if a second value is
   * given it should be zero, since a JntArray resembles a column.
   *
   *
   * @return the joint value at position i, starting from 0
   * @pre 0 != size (ie non-default constructor or resize() called)
   */
  double operator()(unsigned int i, unsigned int j = 0) const;
  /**
   * get_item operator for the joint array, if a second value is
   * given it should be zero, since a JntArray resembles a column.
   *
   *
   * @return the joint value at position i, starting from 0
   * @pre 0 != size (ie non-default constructor or resize() called)
   */
  double& operator()(unsigned int i, unsigned int j = 0);
  /**
   * Returns the number of rows (size) of the array
   */
  unsigned int rows() const;
  /**
   * Returns the number of columns of the array, always 1.
   */
  unsigned int columns() const;

  friend void Add(const JntArray& src1, const JntArray& src2, JntArray& dest);
  friend void Subtract(const JntArray& src1, const JntArray& src2,
                       JntArray& dest);
  friend void Multiply(const JntArray& src, const double& factor,
                       JntArray& dest);
  friend void Divide(const JntArray& src, const double& factor, JntArray& dest);
  friend void MultiplyJacobian(const Jacobian& jac, const JntArray& src,
                               Twist& dest);
  friend void SetToZero(JntArray& array);
  friend bool Equal(const JntArray& src1, const JntArray& src2, double eps);

  friend bool operator==(const JntArray& src1, const JntArray& src2);
};

bool operator==(const JntArray& src1, const JntArray& src2);
// bool operator!=(const JntArray& src1,const JntArray& src2);

/**
 * Function to add two joint arrays, all the arguments must
 * have the same size: A + B = C. This function is
 * aliasing-safe, A or B can be the same array as C.
 *
 * @param src1 A
 * @param src2 B
 * @param dest C
 */
void Add(const JntArray& src1, const JntArray& src2, JntArray& dest);
/**
 * Function to subtract two joint arrays, all the arguments must
 * have the same size: A - B = C. This function is
 * aliasing-safe, A or B can be the same array as C.
 *
 * @param src1 A
 * @param src2 B
 * @param dest C
 */
void Subtract(const JntArray& src1, const JntArray& src2, JntArray& dest);
/**
 * Function to multiply all the array values with a scalar
 * factor: A*b=C. This function is aliasing-safe, A can be the
 * same array as C.
 *
 * @param src A
 * @param factor b
 * @param dest C
 */
void Multiply(const JntArray& src, const double& factor, JntArray& dest);
/**
 * Function to divide all the array values with a scalar
 * factor: A/b=C. This function is aliasing-safe, A can be the
 * same array as C.
 *
 * @param src A
 * @param factor b
 * @param dest C
 */
void Divide(const JntArray& src, const double& factor, JntArray& dest);
/**
 * Function to multiply a KDL::Jacobian with a KDL::JntArray
 * to get a KDL::Twist, it should not be used to calculate the
 * forward velocity kinematics, the solver classes are built
 * for this purpose.
 * J*q = t
 *
 * @param jac J
 * @param src q
 * @param dest t
 * @post dest==Twist::Zero() if 0==src.rows() (ie src is empty)
 */
void MultiplyJacobian(const Jacobian& jac, const JntArray& src, Twist& dest);
/**
 * Function to set all the values of the array to 0
 *
 * @param array
 */
void SetToZero(JntArray& array);
/**
 * Function to check if two arrays are the same with a
 *precision of eps
 *
 * @param src1
 * @param src2
 * @param eps default: epsilon
 * @return true if each element of src1 is within eps of the same
 * element in src2, or if both src1 and src2 have no data (ie 0==rows())
 */
bool Equal(const JntArray& src1, const JntArray& src2, double eps = EPSILON);

class Vector {
 public:
  double data[3]{};

  //! Does not initialise the Vector to zero. use Vector::Zero() or SetToZero
  //! for that
  Vector() { data[0] = data[1] = data[2] = 0.0; }

  //! Constructs a vector out of the three values x, y and z
  Vector(double x, double y, double z);

  //! Assignment operator. The normal copy by value semantics.
  Vector(const Vector& arg);

  //! Assignment operator. The normal copy by value semantics.
  Vector& operator=(const Vector& arg);

  //! Access to elements, range checked when NDEBUG is not set, from 0..2
  double operator()(int index) const;

  //! Access to elements, range checked when NDEBUG is not set, from 0..2
  double& operator()(int index);

  //! Equivalent to double operator()(int index) const
  double operator[](int index) const { return this->operator()(index); }

  //! Equivalent to double& operator()(int index)
  double& operator[](int index) { return this->operator()(index); }

  double x() const;
  double y() const;
  double z() const;
  void x(double val);
  void y(double val);
  void z(double val);

  //! Reverses the sign of the Vector object itself
  void ReverseSign();

  //! subtracts a vector from the Vector object itself
  Vector& operator-=(const Vector& arg);

  //! Adds a vector from the Vector object itself
  Vector& operator+=(const Vector& arg);

  //! Scalar multiplication is defined
  friend Vector operator*(const Vector& lhs, double rhs);
  //! Scalar multiplication is defined
  friend Vector operator*(double lhs, const Vector& rhs);
  //! Scalar division is defined
  friend Vector operator/(const Vector& lhs, double rhs);

  friend Vector operator+(const Vector& lhs, const Vector& rhs);

  friend Vector operator-(const Vector& lhs, const Vector& rhs);
  friend Vector operator*(const Vector& lhs, const Vector& rhs);
  friend Vector operator-(const Vector& arg);
  friend double dot(const Vector& lhs, const Vector& rhs);

  //! To have a uniform operator to put an element to zero, for scalar values
  //! and for objects.
  friend void SetToZero(Vector& v);

  //! @return a zero vector
  static Vector Zero();

  /** Normalizes this vector and returns it norm
   * makes v a unitvector and returns the norm of v.
   * if v is smaller than eps, Vector(1,0,0) is returned with norm 0.
   * if this is not good, check the return value of this method.
   */
  double Normalize(double eps = EPSILON);

  //!    @return the norm of the vector
  double Norm(double eps = EPSILON) const;

  //! do not use operator == because the definition of Equal(.,.) is slightly
  //! different.  It compares whether the 2 arguments are equal in an
  //! eps-interval
  friend bool Equal(const Vector& a, const Vector& b, double eps);

  //! The literal equality operator==(), also identical.
  friend bool operator==(const Vector& a, const Vector& b);
  //! The literal inequality operator!=().
  friend bool operator!=(const Vector& a, const Vector& b);

  friend class Rotation;
  friend class Frame;
};

class Rotation {
 public:
  double data[9]{};

  Rotation() { *this = Rotation::Identity(); }
  Rotation(double Xx, double Yx, double Zx, double Xy, double Yy, double Zy,
           double Xz, double Yz, double Zz);
  Rotation(const Vector& x, const Vector& y, const Vector& z);

  Rotation(const Rotation& arg);

  Rotation& operator=(const Rotation& arg);

  //!  Defines a multiplication R*V between a Rotation R and a Vector V.
  //! Complexity : 9M+6A
  Vector operator*(const Vector& v) const;

  //!    Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set
  double& operator()(int i, int j);

  //!    Access to elements 0..2,0..2, bounds are checked when NDEBUG is not set
  double operator()(int i, int j) const;

  friend Rotation operator*(const Rotation& lhs, const Rotation& rhs);

  //! Sets the value of *this to its inverse.
  void SetInverse();

  //! Gives back the inverse rotation matrix of *this.
  [[nodiscard]] Rotation Inverse() const;

  //! The same as R.Inverse()*v but more efficient.
  [[nodiscard]] Vector Inverse(const Vector& v) const;

  //! The same as R.Inverse()*arg but more efficient.
  [[nodiscard]] Wrench Inverse(const Wrench& arg) const;

  //! The same as R.Inverse()*arg but more efficient.
  [[nodiscard]] Twist Inverse(const Twist& arg) const;

  //! Gives back an identity rotaton matrix
  [[nodiscard]] static Rotation Identity();

  // = Rotations
  //! The Rot... static functions give the value of the appropriate rotation
  //! matrix back.
  [[nodiscard]] static Rotation RotX(double angle);
  //! The Rot... static functions give the value of the appropriate rotation
  //! matrix back.
  [[nodiscard]] static Rotation RotY(double angle);
  //! The Rot... static functions give the value of the appropriate rotation
  //! matrix back.
  [[nodiscard]] static Rotation RotZ(double angle);
  //! The DoRot... functions apply a rotation R to *this,such that *this = *this
  //! * Rot.. DoRot... functions are only defined when they can be executed more
  //! efficiently
  void DoRotX(double angle);
  //! The DoRot... functions apply a rotation R to *this,such that *this = *this
  //! * Rot.. DoRot... functions are only defined when they can be executed more
  //! efficiently
  void DoRotY(double angle);
  //! The DoRot... functions apply a rotation R to *this,such that *this = *this
  //! * Rot.. DoRot... functions are only defined when they can be executed more
  //! efficiently
  void DoRotZ(double angle);

  //! Along an arbitrary axes.  It is not necessary to normalize rotvec.
  //! returns identity rotation matrix in the case that the norm of rotvec
  //! is to small to be used.
  // @see Rot2 if you want to handle this error in another way.
  static Rotation Rot(const Vector& rotvec, double angle);

  //! Along an arbitrary axes.  rotvec should be normalized.
  static Rotation Rot2(const Vector& rotvec, double angle);

  //! Returns a vector with the direction of the equiv. axis
  //! and its norm is angle
  [[nodiscard]] Vector GetRot() const;

  /** Returns the rotation angle around the equiv. axis
   * @param axis the rotation axis is returned in this variable
   * @param eps :  in the case of angle == 0 : rot axis is undefined and chosen
   *                                         to be +/- Z-axis
   *               in the case of angle == PI : 2 solutions, positive
   * Z-component of the axis is chosen.
   * @result returns the rotation angle (between [0..PI] )
   */
  double GetRotAngle(Vector& axis, double eps = EPSILON) const;

  /**     Gives back a rotation matrix specified with EulerZYZ convention :
   *       - First rotate around Z with alfa,
   *       - then around the new Y with beta,
   *	     - then around new Z with gamma.
   *  Invariants:
   *  	- EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PHI, -beta, gamma +/-
   * PI)
   *  	- (angle + 2*k*PI)
   **/
  static Rotation EulerZYZ(double Alfa, double Beta, double Gamma);

  /** Gives back the EulerZYZ convention description of the rotation matrix :
   First rotate around Z with alpha,
   then around the new Y with beta, then around
   new Z with gamma.

   Variables are bound by:
   - (-PI <  alpha  <= PI),
   - (0   <= beta  <= PI),
   - (-PI <  gamma <= PI)

   if beta==0 or beta==PI, then alpha and gamma are not unique, in this case
   gamma is chosen to be zero. Invariants:
     - EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, -beta, gamma +/- PI)
     - angle + 2*k*PI
   */
  void GetEulerZYZ(double& alpha, double& beta, double& gamma) const;

  //! Gives back a rotation matrix specified with Quaternion convention
  //! the norm of (x,y,z,w) should be equal to 1
  static Rotation Quaternion(double x, double y, double z, double w);

  //! Get the quaternion of this matrix
  //! \post the norm of (x,y,z,w) is 1
  void GetQuaternion(double& x, double& y, double& z, double& w) const;

  /**
   *
   * Gives back a rotation matrix specified with RPY convention:
   * first rotate around X with roll, then around the
   *              old Y with pitch, then around old Z with yaw
   *
   * Invariants:
   *  - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
   *  - angles + 2*k*PI
   */
  static Rotation RPY(double roll, double pitch, double yaw);

  /**  Gives back a vector in RPY coordinates, variables are bound by
       -  -PI <= roll <= PI
       -   -PI <= Yaw  <= PI
       -  -PI/2 <= PITCH <= PI/2

           convention :
           - first rotate around X with roll,
           - then around the old Y with pitch,
           - then around old Z with yaw

           if pitch == PI/2 or pitch == -PI/2, multiple solutions for gamma and
  alpha exist.  The solution where roll==0 is chosen.

           Invariants:
           - RPY(roll,pitch,yaw) == RPY( roll +/- PI, PI-pitch, yaw +/- PI )
           - angles + 2*k*PI

  **/
  void GetRPY(double& roll, double& pitch, double& yaw) const;

  /**  EulerZYX constructs a Rotation from the Euler ZYX parameters:
   *   -  First rotate around Z with alfa,
   *   - then around the new Y with beta,
   *   - then around new X with gamma.
   *
   *  Closely related to RPY-convention.
   *
   *  Invariants:
   *  	- EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, PI-beta, gamma
   * +/- PI)
   *  	- (angle + 2*k*PI)
   **/
  static Rotation EulerZYX(double Alfa, double Beta, double Gamma) {
    return RPY(Gamma, Beta, Alfa);
  }

  /**   GetEulerZYX gets the euler ZYX parameters of a rotation :
   *   First rotate around Z with alfa,
   *   then around the new Y with beta, then around
   *   new X with gamma.
   *
   *  Range of the results of GetEulerZYX :
   *  -  -PI <= alfa <= PI
   *  -   -PI <= gamma <= PI
   *  -  -PI/2 <= beta <= PI/2
   *
   *  if beta == PI/2 or beta == -PI/2, multiple solutions for gamma and alpha
   * exist.  The solution where gamma==0 is chosen.
   *
   *
   *  Invariants:
   *  	- EulerZYX(alpha,beta,gamma) == EulerZYX(alpha +/- PI, PI-beta, gamma
   * +/- PI)
   *  	- and also (angle + 2*k*PI)
   *
   *  Closely related to RPY-convention.
   **/
  void GetEulerZYX(double& Alfa, double& Beta, double& Gamma) const {
    GetRPY(Gamma, Beta, Alfa);
  }

  //! Transformation of the base to which the twist is expressed.
  //! Complexity : 18M+12A
  //! @see Frame*Twist for a transformation that also transforms
  //! the velocity reference point.
  Twist operator*(const Twist& arg) const;

  //! Transformation of the base to which the wrench is expressed.
  //! Complexity : 18M+12A
  //! @see Frame*Wrench for a transformation that also transforms
  //! the force reference point.
  Wrench operator*(const Wrench& arg) const;

  //! Access to the underlying unitvectors of the rotation matrix
  [[nodiscard]] Vector UnitX() const { return {data[0], data[3], data[6]}; }

  //! Access to the underlying unitvectors of the rotation matrix
  void UnitX(const Vector& X) {
    data[0] = X(0);
    data[3] = X(1);
    data[6] = X(2);
  }

  //! Access to the underlying unitvectors of the rotation matrix
  [[nodiscard]] Vector UnitY() const { return {data[1], data[4], data[7]}; }

  //! Access to the underlying unitvectors of the rotation matrix
  void UnitY(const Vector& X) {
    data[1] = X(0);
    data[4] = X(1);
    data[7] = X(2);
  }

  //! Access to the underlying unitvectors of the rotation matrix
  [[nodiscard]] Vector UnitZ() const { return {data[2], data[5], data[8]}; }

  //! Access to the underlying unitvectors of the rotation matrix
  void UnitZ(const Vector& X) {
    data[2] = X(0);
    data[5] = X(1);
    data[8] = X(2);
  }

  //! do not use operator == because the definition of Equal(.,.) is slightly
  //! different.  It compares whether the 2 arguments are equal in an
  //! eps-interval
  friend bool Equal(const Rotation& a, const Rotation& b, double eps);

  //! The literal equality operator==(), also identical.
  friend bool operator==(const Rotation& a, const Rotation& b);
  //! The literal inequality operator!=()
  friend bool operator!=(const Rotation& a, const Rotation& b);

  friend class Frame;
};
bool operator==(const Rotation& a, const Rotation& b);
bool Equal(const Rotation& a, const Rotation& b, double eps = EPSILON);

class Frame {
 public:
  Vector p;    //!< origine of the Frame
  Rotation M;  //!< Orientation of the Frame

 public:
  Frame(const Rotation& R, const Vector& V);

  //! The rotation matrix defaults to identity
  explicit Frame(const Vector& V);
  //! The position matrix defaults to zero
  explicit Frame(const Rotation& R);

  Frame() = default;
  //! The copy constructor. Normal copy by value semantics.
  Frame(const Frame& arg);

  //! Reads data from an double array
  //\TODO should be formulated as a constructor
  void Make4x4(double* d);

  //!  Treats a frame as a 4x4 matrix and returns element i,j
  //!  Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set
  double operator()(int i, int j);

  //!  Treats a frame as a 4x4 matrix and returns element i,j
  //!    Access to elements 0..3,0..3, bounds are checked when NDEBUG is not set
  double operator()(int i, int j) const;

  // = Inverse
  //! Gives back inverse transformation of a Frame
  [[nodiscard]] Frame Inverse() const;

  //! The same as p2=R.Inverse()*p but more efficient.
  [[nodiscard]] Vector Inverse(const Vector& arg) const;

  //! The same as p2=R.Inverse()*p but more efficient.
  [[nodiscard]] Wrench Inverse(const Wrench& arg) const;

  //! The same as p2=R.Inverse()*p but more efficient.
  [[nodiscard]] Twist Inverse(const Twist& arg) const;

  //! Normal copy-by-value semantics.
  Frame& operator=(const Frame& arg);

  //! Transformation of the base to which the vector
  //! is expressed.
  Vector operator*(const Vector& arg) const;

  //! Transformation of both the force reference point
  //! and of the base to which the wrench is expressed.
  //! look at Rotation*Wrench operator for a transformation
  //! of only the base to which the twist is expressed.
  //!
  //! Complexity : 24M+18A
  Wrench operator*(const Wrench& arg) const;

  //! Transformation of both the velocity reference point
  //! and of the base to which the twist is expressed.
  //! look at Rotation*Twist for a transformation of only the
  //! base to which the twist is expressed.
  //!
  //! Complexity : 24M+18A
  Twist operator*(const Twist& arg) const;

  //! Composition of two frames.
  friend Frame operator*(const Frame& lhs, const Frame& rhs);

  //! @return the identity transformation
  //! Frame(Rotation::Identity(),Vector::Zero()).
  static Frame Identity();

  //! The twist <t_this> is expressed wrt the current
  //! frame.  This frame is integrated into an updated frame with
  //! <samplefrequency>.  Very simple first order integration rule.
  void Integrate(const Twist& t_this, double frequency);

  /*
  // DH_Craig1989 : constructs a transformationmatrix
  // T_link(i-1)_link(i) with the Denavit-Hartenberg convention as
  // described in the Craigs book: Craig, J. J.,Introduction to
  // Robotics: Mechanics and Control, Addison-Wesley,
  // isbn:0-201-10326-5, 1986.
  //
  // Note that the frame is a redundant way to express the information
  // in the DH-convention.
  // \verbatim
  // Parameters in full : a(i-1),alpha(i-1),d(i),theta(i)
  //
  //  axis i-1 is connected by link i-1 to axis i numbering axis 1
  //  to axis n link 0 (immobile base) to link n
  //
  //  link length a(i-1) length of the mutual perpendicular line
  //  (normal) between the 2 axes.  This normal runs from (i-1) to
  //  (i) axis.
  //
  //  link twist alpha(i-1): construct plane perpendicular to the
  //  normal project axis(i-1) and axis(i) into plane angle from
  //  (i-1) to (i) measured in the direction of the normal
  //
  //  link offset d(i) signed distance between normal (i-1) to (i)
  //  and normal (i) to (i+1) along axis i joint angle theta(i)
  //  signed angle between normal (i-1) to (i) and normal (i) to
  //  (i+1) along axis i
  //
  //   First and last joints : a(0)= a(n) = 0
  //   alpha(0) = alpha(n) = 0
  //
  //   PRISMATIC : theta(1) = 0 d(1) arbitrarily
  //
  //   REVOLUTE : theta(1) arbitrarily d(1) = 0
  //
  //   Not unique : if intersecting joint axis 2 choices for normal
  //   Frame assignment of the DH convention : Z(i-1) follows axis
  //   (i-1) X(i-1) is the normal between axis(i-1) and axis(i)
  //   Y(i-1) follows out of Z(i-1) and X(i-1)
  //
  //     a(i-1)     = distance from Z(i-1) to Z(i) along X(i-1)
  //     alpha(i-1) = angle between Z(i-1) to Z(i) along X(i-1)
  //     d(i)       = distance from X(i-1) to X(i) along Z(i)
  //     theta(i)   = angle between X(i-1) to X(i) along X(i)
  // \endverbatim
  */
  static Frame DH_Craig1989(double a, double alpha, double d, double theta);

  // DH : constructs a transformationmatrix T_link(i-1)_link(i) with
  // the Denavit-Hartenberg convention as described in the original
  // publictation: Denavit, J. and Hartenberg, R. S., A kinematic
  // notation for lower-pair mechanisms based on matrices, ASME
  // Journal of Applied Mechanics, 23:215-221, 1955.

  static Frame DH(double a, double alpha, double d, double theta);

  //! do not use operator == because the definition of Equal(.,.) is slightly
  //! different.  It compares whether the 2 arguments are equal in an
  //! eps-interval
  friend bool Equal(const Frame& a, const Frame& b, double eps);

  //! The literal equality operator==(), also identical.
  friend bool operator==(const Frame& a, const Frame& b);
  //! The literal inequality operator!=().
  friend bool operator!=(const Frame& a, const Frame& b);
};

class Wrench {
 public:
  Vector
      force;  //!< Force that is applied at the origin of the current ref frame
  Vector torque;  //!< Torque that is applied at the origin of the current ref
                  //!< frame
 public:
  //! Does  initialise force and torque to zero via the underlying constructor
  //! of Vector
  Wrench() : force(), torque() {};
  Wrench(const Vector& _force, const Vector& _torque)
      : force(_force), torque(_torque) {};

  // = Operators
  Wrench& operator-=(const Wrench& arg);
  Wrench& operator+=(const Wrench& arg);

  //! index-based access to components, first force(0..2), then torque(3..5)
  double& operator()(int i);

  //! index-based access to components, first force(0..2), then torque(3..5)
  //! for use with a const Wrench
  double operator()(int i) const;

  double operator[](int index) const { return this->operator()(index); }

  double& operator[](int index) { return this->operator()(index); }

  //! Scalar multiplication
  friend Wrench operator*(const Wrench& lhs, double rhs);
  //! Scalar multiplication
  friend Wrench operator*(double lhs, const Wrench& rhs);
  //! Scalar division
  friend Wrench operator/(const Wrench& lhs, double rhs);

  friend Wrench operator+(const Wrench& lhs, const Wrench& rhs);
  friend Wrench operator-(const Wrench& lhs, const Wrench& rhs);

  //! An unary - operator
  friend Wrench operator-(const Wrench& arg);

  //! Sets the Wrench to Zero, to have a uniform function that sets an object or
  //! double to zero.
  friend void SetToZero(Wrench& v);

  //! @return a zero Wrench
  static Wrench Zero();

  //! Reverses the sign of the current Wrench
  void ReverseSign();

  //! Changes the reference point of the wrench.
  //! The vector v_base_AB is expressed in the same base as the twist
  //! The vector v_base_AB is a vector from the old point to
  //! the new point.
  //!
  //! Complexity : 6M+6A
  [[nodiscard]] Wrench RefPoint(const Vector& v_base_AB) const;

  //! do not use operator == because the definition of Equal(.,.) is slightly
  //! different.  It compares whether the 2 arguments are equal in an
  //! eps-interval
  friend bool Equal(const Wrench& a, const Wrench& b, double eps);

  //! The literal equality operator==(), also identical.
  friend bool operator==(const Wrench& a, const Wrench& b);
  //! The literal inequality operator!=().
  friend bool operator!=(const Wrench& a, const Wrench& b);

  friend class Rotation;
  friend class Frame;
};

class Twist {
 public:
  Vector vel;  //!< The velocity of that point
  Vector rot;  //!< The rotational velocity of that point.
 public:
  //! The default constructor initialises to Zero via the constructor of Vector.
  Twist() : vel(), rot() {};

  Twist(const Vector& _vel, const Vector& _rot) : vel(_vel), rot(_rot) {};

  Twist& operator-=(const Twist& arg);
  Twist& operator+=(const Twist& arg);
  //! index-based access to components, first vel(0..2), then rot(3..5)
  double& operator()(int i);

  //! index-based access to components, first vel(0..2), then rot(3..5)
  //! For use with a const Twist
  double operator()(int i) const;

  double operator[](int index) const { return this->operator()(index); }

  double& operator[](int index) { return this->operator()(index); }

  friend Twist operator*(const Twist& lhs, double rhs);
  friend Twist operator*(double lhs, const Twist& rhs);
  friend Twist operator/(const Twist& lhs, double rhs);
  friend Twist operator+(const Twist& lhs, const Twist& rhs);
  friend Twist operator-(const Twist& lhs, const Twist& rhs);
  friend Twist operator-(const Twist& arg);
  friend double dot(const Twist& lhs, const Wrench& rhs);
  friend double dot(const Wrench& rhs, const Twist& lhs);
  friend void SetToZero(Twist& v);
  /// Spatial cross product for 6d motion vectors, beware all of them have to be
  /// expressed in the same reference frame/point
  friend Twist operator*(const Twist& lhs, const Twist& rhs);
  /// Spatial cross product for 6d force vectors, beware all of them have to be
  /// expressed in the same reference frame/point
  friend Wrench operator*(const Twist& lhs, const Wrench& rhs);

  //! @return a zero Twist : Twist(Vector::Zero(),Vector::Zero())
  static Twist Zero();

  //! Reverses the sign of the twist
  void ReverseSign();

  //! Changes the reference point of the twist.
  //! The vector v_base_AB is expressed in the same base as the twist
  //! The vector v_base_AB is a vector from the old point to
  //! the new point.
  //!
  //! Complexity : 6M+6A
  [[nodiscard]] Twist RefPoint(const Vector& v_base_AB) const;

  //! do not use operator == because the definition of Equal(.,.) is slightly
  //! different.  It compares whether the 2 arguments are equal in an
  //! eps-interval
  friend bool Equal(const Twist& a, const Twist& b, double eps);

  //! The literal equality operator==(), also identical.
  friend bool operator==(const Twist& a, const Twist& b);
  //! The literal inequality operator!=().
  friend bool operator!=(const Twist& a, const Twist& b);

  // = Friends
  friend class Rotation;
  friend class Frame;
};

class Jacobian {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<double, 6, Eigen::Dynamic> data;
  Jacobian();
  explicit Jacobian(unsigned int nr_of_columns);
  Jacobian(const Jacobian& arg);

  /// Allocates memory for new size (can break realtime behavior)
  void resize(unsigned int newNrOfColumns);

  /// Allocates memory if size of this and argument is different
  Jacobian& operator=(const Jacobian& arg);

  bool operator==(const Jacobian& arg) const;
  bool operator!=(const Jacobian& arg) const;

  friend bool Equal(const Jacobian& a, const Jacobian& b, double eps);

  ~Jacobian();

  double operator()(unsigned int i, unsigned int j) const;
  double& operator()(unsigned int i, unsigned int j);
  [[nodiscard]] unsigned int rows() const;
  [[nodiscard]] unsigned int columns() const;

  friend void SetToZero(Jacobian& jac);

  friend bool changeRefPoint(const Jacobian& src1, const Vector& base_AB,
                             Jacobian& dest);
  friend bool changeBase(const Jacobian& src1, const Rotation& rot,
                         Jacobian& dest);
  friend bool changeRefFrame(const Jacobian& src1, const Frame& frame,
                             Jacobian& dest);

  [[nodiscard]] Twist getColumn(unsigned int i) const;
  void setColumn(unsigned int i, const Twist& t);

  void changeRefPoint(const Vector& base_AB);
  void changeBase(const Rotation& rot);
  void changeRefFrame(const Frame& frame);
};

bool changeRefPoint(const Jacobian& src1, const Vector& base_AB,
                    Jacobian& dest);
bool changeBase(const Jacobian& src1, const Rotation& rot, Jacobian& dest);
bool changeRefFrame(const Jacobian& src1, const Frame& frame, Jacobian& dest);

}  // namespace rocos