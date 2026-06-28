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

#include <cmath>
#include <cstdlib>

namespace rocos {

#ifdef __GNUC__
    // so that sin,cos can be overloaded and complete
    // resolution of overloaded functions work.
    using ::sin;
    using ::cos;
    using ::exp;
    using ::log;
    using ::sin;
    using ::cos;
    using ::tan;
    using ::sinh;
    using ::cosh;
    using ::pow;
    using ::sqrt;
    using ::atan;
    using ::hypot;
    using ::asin;
    using ::acos;
    using ::tanh;
    using ::atan2;
#endif
#ifndef __GNUC__
    //only real solution : get Rall1d and varia out of namespaces.
    #pragma warning (disable:4786)

    inline double sin(double a) {
        return ::sin(a);
    }

    inline double cos(double a) {
        return ::cos(a);
    }
    inline double exp(double a) {
        return ::exp(a);
    }
    inline double log(double a) {
        return ::log(a);
    }
    inline double tan(double a) {
        return ::tan(a);
    }
    inline double cosh(double a) {
        return ::cosh(a);
    }
    inline double sinh(double a) {
        return ::sinh(a);
    }
    inline double sqrt(double a) {
        return ::sqrt(a);
    }
    inline double atan(double a) {
        return ::atan(a);
    }
    inline double acos(double a) {
        return ::acos(a);
    }
    inline double asin(double a) {
        return ::asin(a);
    }
    inline double tanh(double a) {
        return ::tanh(a);
    }
    inline double pow(double a,double b) {
        return ::pow(a,b);
    }
    inline double atan2(double a,double b) {
        return ::atan2(a,b);
    }
#endif

#if (__cplusplus > 199711L)
using std::isnan;
#endif

/**
 * Auxiliary class for argument types (Trait-template class )
 *
 * Is used to pass doubles by value, and arbitrary objects by const reference.
 * This is TWICE as fast (2 x less memory access) and avoids bugs in VC6++ concerning
 * the assignment of the result of intrinsic functions to const double&-typed variables,
 * and optimization on.
 */
template <class T>
class TI
{
    public:
        typedef const T& Arg; //!< Arg is used for passing the element to a function.
};

template <>
class TI<double> {
public:
    typedef double Arg;
};

template <>
class TI<int> {
public:
    typedef int Arg;
};


//!
inline constexpr int STREAMBUFFERSIZE = 10000;

//! maximal length of a file name
inline constexpr int MAXLENFILENAME = 255;

//! the value of pi
inline constexpr double PI = 3.141592653589793238462643383279502884;

//! the value of pi/2
inline constexpr double PI_2 = 1.570796326794896619231321691639751442;

//! the value of pi/4
inline constexpr double PI_4 = 0.785398163397448309615660845819875721;

//! the value pi/180
inline constexpr double DEG2RAD = 0.017453292519943295769236907684886127;

//! the value 180/pi
inline constexpr double RAD2DEG = 57.29577951308232087679815481410517033;

//! default precision while comparing with Equal(..,..) functions. Initialized
//! at 0.0000001.
inline constexpr double EPSILON = 1e-6;

#ifndef _MFC_VER
#undef max
inline double max(double a, double b) {
  if (b < a)
    return a;
  else
    return b;
}

#undef min
inline double min(double a, double b) {
  if (b < a)
    return b;
  else
    return a;
}
#endif

#ifdef _MSC_VER
// #pragma inline_depth( 255 )
// #pragma inline_recursion( on )
#define INLINE __forceinline
// #define INLINE inline
#else
#define INLINE inline
#endif

inline double LinComb(double alfa, double a, double beta, double b) {
  return alfa * a + beta * b;
}

inline void LinCombR(double alfa, double a, double beta, double b,
                     double& result) {
  result = alfa * a + beta * b;
}

//! to uniformly set double, RNDouble,Vector,... objects to zero in
//! template-classes
inline void SetToZero(double& arg) { arg = 0; }

//! to uniformly set double, RNDouble,Vector,... objects to the identity element
//! in template-classes
inline void SetToIdentity(double& arg) { arg = 1; }

inline double sign(double arg) { return (arg < 0) ? (-1) : (1); }

inline double sqr(double arg) { return arg * arg; }
inline double Norm(double arg) { return fabs((double)arg); }

#if defined __WIN32__ && !defined __GNUC__
inline double hypot(double y, double x) { return ::_hypot(y, x); }
inline double abs(double x) { return ::fabs(x); }
#endif

// compares whether 2 doubles are equal in an eps-interval.
// Does not check whether a or b represents numbers
// On VC6, if a/b is -INF, it returns false;
inline bool Equal(double a, double b, double eps = EPSILON) {
  double tmp = (a - b);
  return ((eps > tmp) && (tmp > -eps));
}

inline void random(double& a) { a = 1.98 * rand() / (double)RAND_MAX - 0.99; }

inline void posrandom(double& a) {
  a = 0.001 + 0.99 * rand() / (double)RAND_MAX;
}

inline double diff(double a, double b, double dt) { return (b - a) / dt; }
// inline float diff(float a,float b,double dt) {
// return (b-a)/dt;
// }
inline double addDelta(double a, double da, double dt) { return a + da * dt; }

}  // namespace rocos
