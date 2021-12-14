

#ifndef INTERPOLATE_H_
#define INTERPOLATE_H_


#pragma once
//#include "matrix.hpp"
#include "robotMath.h"

#include <cmath>

namespace rocos {

    enum PULSE_TYPE {
        JERK,
        ACC,
        VEL
    };

    class PULSE        // 方波轨迹
    {
    public:
        PULSE(PULSE_TYPE type, double cycle, double ratio, double amp) {
            _ratio = ratio;
            _cycle = cycle;
            _type = type;
            _amp = amp;
        }

        ~PULSE() {}

    private:
        double _amp;            // 方波fuzhi
        double _ratio;        // 方波占空比
        double _cycle;        // 方波周期
        PULSE_TYPE _type;    // 速度方波、加速度方波、冲击方波
    public:
        double pos(double time);

        double vel(double time);
        //double acc(double time);
        //double jerk(double time);
    };

    enum ProfileType {
        unDefine,            // 未指定类型
        trapezoid,            // 速度T型波
        doubleS,            // 速度S型波
        polynomial_linear_T,
        polynomial_3rd_T,    // 3次多项式
        polynomial_5th_T,    // 5次多项式
        harmonic_T,            // 三角谐波
        cycloidal_T,        // 三角摆线
        elliptic_T,            // 三角椭圆
        b_spline_T            // B样条
    };

// 接口
    class R_INTERP_BASE {
    public:
        R_INTERP_BASE() {};

        virtual ~R_INTERP_BASE() {};

        // returns the current profile type
        virtual ProfileType profileType() const = 0;

        /// returns true if all time intervals are non negative
        /// and all pos vel acc values are inside the limits.
        virtual bool isValidMovement() const = 0;

        /// get total duration of the trajectory.
        virtual double getDuration() const = 0;

        /// get the position, velocity and acceleration at passed time >= 0
        virtual double pos(double t) const = 0;

        virtual double vel(double t) const = 0;

        virtual double acc(double t) const = 0;

        virtual double jerk(double t) const { return 0; }

        // get the maximum velocity acceleration and jerk at of the profile
        virtual double max_vel() const = 0;

        virtual double max_acc() const = 0;

        virtual double max_jerk() const = 0;

        /// scale a planned profile to a longer duration
        virtual double scaleToDuration(double newDuration) { return newDuration; };
    };

// 速度T型波
    class Trapezoid : public R_INTERP_BASE {
    public:

        Trapezoid() {
            _direction = 1;
            _plannedProfile = false;
        }

        ~Trapezoid() {};

        virtual double getDuration() const;

        virtual double pos(double t) const;

        virtual double vel(double t) const;

        virtual double acc(double t) const;

        virtual double jerk(double t) const;

        virtual bool isValidMovement() const;

        virtual double scaleToDuration(double newDuration);

        virtual ProfileType profileType() const { return trapezoid; }

        virtual double max_vel() const;

        virtual double max_acc() const;

        virtual double max_jerk() const { return 0.0; };

        void planTrapezoidProfile(double t0, double p0, double pf,
                                  double v0, double vf, double v_limit, double a_limit);

    private:
        void trapezoid_check_T(double t0, double p0, double pf,
                               double v0, double vf, double v_limit, double a_limit);

    private:
        double _x[4];
        double _v[4];            // ?[0] is start condition, ?[3] is end condition.
        double _t[4];            // t[i] is point in time, not interval
        double _a[4];            // a[i] is acceleration between t[i-1] and t[i]

        int _direction;            // direction of the movement
        bool _plannedProfile;    // flag indication whether a profile was computed

    };

// 速度S型波
    class DoubleS : public R_INTERP_BASE {
    public:

        DoubleS() {
            _direction = 1;
            _plannedProfile = false;
        }

        ~DoubleS() {};

        virtual double getDuration() const;

        virtual double pos(double t) const;

        virtual double vel(double t) const;

        virtual double acc(double t) const;

        virtual double jerk(double t) const;

        virtual bool isValidMovement() const;

        virtual double scaleToDuration(double newDuration);

        virtual ProfileType profileType() const { return doubleS; }

        virtual double max_vel() const;

        virtual double max_acc() const;

        virtual double max_jerk() const { return fabs(_j[1]); };

        void planDoubleSProfile(double t0, double p0, double pf,
                                double v0, double vf, double v_limit, double a_limit, double j_limit);

    private:
        void doubleS_check_T(double t0, double p0, double pf,
                             double v0, double vf, double v_limit, double a_limit, double j_limit);

    private:
        double _x[8];
        double _v[8];            // ?[0] is start condition, ?[3] is end condition.
        double _t[8];            // t[i] is point in time, not interval
        double _a[8];            // a[i] is acceleration at switch time
        double _j[8];            // j[i] is jerk between t[i-1] and t[i]
        int _direction;            // direction of the movement
        bool _plannedProfile;    // flag indication whether a profile was computed
    };

// 多项式，3次，5次
    class Polynomial : public R_INTERP_BASE {

    public:

        Polynomial() { _plannedProfile = false; }

        ~Polynomial() {};

        virtual double getDuration() const;

        virtual double pos(double t) const;

        virtual double vel(double t) const;

        virtual double acc(double t) const;

        virtual double jerk(double t) const;

        virtual bool isValidMovement() const;

        virtual double scaleToDuration(double newDuration);

        virtual ProfileType profileType() const { return _pType; }

        virtual double max_vel() const;

        virtual double max_acc() const;

        virtual double max_jerk() const;

        // 定时
        void planLinearProfileT(double t0, double tf, double p0, double pf);

        void plan3rdProfileT(double t0, double tf, double p0, double pf, double v0, double vf);

        void plan5thProfileT(double t0, double tf, double p0, double pf, double v0, double vf, double a0, double af);

    private:

        double _x[2];
        double _v[2];                // ?[0] is start condition, ?[3] is end condition.
        double _t[2];
        double _a[2];
        ProfileType _pType;            // 3rd or 5th polynomial
        bool _plannedProfile;        // flag indication whether a profile was computed
        double _coefficient[6];        // Coefficients of the polynomial
    };

// 三角函数 harmonic cycloidal elliptic
    class Trigonometric : public R_INTERP_BASE {

    public:

        ~Trigonometric() {};

        Trigonometric() : _plannedProfile(false), _n(1.0) {};

        virtual double getDuration() const;

        virtual double pos(double t) const;

        virtual double vel(double t) const;

        virtual double acc(double t) const;

        virtual double jerk(double t) const;

        virtual bool isValidMovement() const;

        virtual double scaleToDuration(double newDuration);

        virtual ProfileType profileType() const { return _pType; }

        virtual double max_vel() const;

        virtual double max_acc() const;

        virtual double max_jerk() const;

        // 定时
        void planHarmonicProfileT(double p0, double pf, double t0, double tf);

        void planCycloidalProfileT(double p0, double pf, double t0, double tf);

        void planEllipticProfileT(double p0, double pf, double t0, double tf, double alpha);

    private:

        double _n;                    // The elliptical curvature
        double _x[2];                // ?[0] is start condition, ?[3] is end condition.
        double _t[2];
        ProfileType _pType;            // profile type
        bool _plannedProfile;        // flag indication whether a profile was computed
    };

// 通用插补
    class R_INTERP {
    public:

        // constructor
        R_INTERP() {
            _pType = unDefine;
            _tStart = 0.0;
            _tTerminal = 0.0;
            _pStart = 0.0;
            _pTarget = 0.0;
            _vStart = 0.0;
            _vTarget = 0.0;
            _aStart = 0.0;
            _aTarget = 0.0;
            _v_limit = 0.0;
            _a_limit = 0.0;
            _j_limit = 0.0;
            _doubleS = nullptr;
            _trapezoid = nullptr;
            _polynomial = nullptr;
            _trigonometric = nullptr;
            _pCurProfileType = nullptr;
        }

        // destructor
        ~R_INTERP() {
            _doubleS = nullptr;
            _trapezoid = nullptr;
            _polynomial = nullptr;
            _trigonometric = nullptr;
            _pCurProfileType = nullptr;
        }

        // set limit values for movement parameters
        void setLimit(double vel_Limit, double acc_Limit, double jerk_Limit) {
            _v_limit = vel_Limit;
            _a_limit = acc_Limit;
            _j_limit = jerk_Limit;
        }

        void setLimit(double vel_Limit, double acc_Limit) { setLimit(vel_Limit, acc_Limit, acc_Limit * 3.0); }

        void setLimit(double vel_Limit) { setLimit(vel_Limit, vel_Limit * 3.0); }

        // set target values for movement parameters
        void setTarget(double pf, double vf, double af) {
            _pTarget = pf;
            _vTarget = vf;
            _aTarget = af;
        }

        void setTarget(double pf, double vf) { setTarget(pf, vf, 0.0); }

        void setTarget(double pf) { setTarget(pf, 0.0, 0.0); }

        // set start values for movement parameters
        void setStart(double p0, double v0, double a0) {
            _pStart = p0;
            _vStart = v0;
            _aStart = a0;
        }

        void setStart(double p0, double v0) { setStart(p0, v0, 0.0); }

        void setStart(double p0) { setStart(p0, 0.0, 0.0); }

        // set the start time
        void setStartTime(double t0) { _tStart = t0; }

        // for some profile type, like TRIGONOMETRIC,
        //   terminal time needed to be given
        void setTargetTime(double tf) { _tTerminal = tf; }

        // set the profile type for movement parameters
        void setProfileType(ProfileType profile) { _pType = profile; }

        // Returns the duration the full movement will need.
        double getDuration() const { return _pCurProfileType->getDuration(); }

        // returns true if all time intervals are non negative
        // and all pos vel acc values are inside the limits.
        bool isValidMovement() const { return _pCurProfileType->isValidMovement(); }

        // Scales an already planned profile to a longer duration.
        double scaleToDuration(double newDuration) { return _pCurProfileType->scaleToDuration(newDuration); }

        // get the position, velocity and acceleration at passed time >= 0
        double pos(double t) const { return _pCurProfileType->pos(t); }

        double vel(double t) const { return _pCurProfileType->vel(t); }

        double acc(double t) const { return _pCurProfileType->acc(t); }

        double jerk(double t) const { return _pCurProfileType->jerk(t); }

        /// plan the time optimal profile
        bool planProfile();

        bool planDoubleSPorfile(double t0, double p0, double pf,
                                double v0, double vf, double v_limit, double a_limit, double j_limit);

        bool planTrapezoidProfile(double t0, double p0, double pf,
                                  double v0, double vf, double v_limit, double a_limit);

        bool plan3rdProfileT(double t0, double tf, double p0,
                             double pf, double v0, double vf);

        bool plan5thProfileT(double t0, double tf, double p0,
                             double pf, double v0, double vf, double a0, double af);


    private:

        ProfileType _pType;

        R_INTERP(const R_INTERP &);

        DoubleS *_doubleS;
        Trapezoid *_trapezoid;
        Polynomial *_polynomial;
        Trigonometric *_trigonometric;
        R_INTERP_BASE *_pCurProfileType;

        double _pTarget;    ///< target position
        double _vTarget;    ///< target velocity
        double _aTarget;    ///< target acceleration

        double _pStart;        ///< start position
        double _vStart;        ///< start velocity
        double _aStart;        ///< start acceleration

        double _tStart;        ///< start time
        double _tTerminal;    ///< terminal time

        double _v_limit;    ///< limit for velocity
        double _a_limit;    ///< limit for acceleration
        double _j_limit;    ///< limit for jerk
    };

} // namespace rocos

#endif /*INTERPOLATE_H_*/