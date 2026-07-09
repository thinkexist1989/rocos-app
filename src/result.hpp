#pragma once

#include <ostream>
#include <string>

namespace rocos {
    enum class Result : int {
        PlanFinished = 1000,


        NoError = 0,

        // API套接字错误
        WsastartFail = -1001,
        CreateSocketFail = -1002,
        BindPortFail = -1003,
        SocketReadFail = -1004,
        SocketTimeout = -1005,
        RecvfromFail = -1006,
        SendtoFail = -1007,
        LostHeartbeat = -1008,
        LostRobotState = -1009,
        GetDhFailed = -1010,
        ReleaseBrakeFailed = -1011,
        HoldBrakeFailed = -1012,
        IpAddressNotRegister = -1013,
        RobotArmOverNumber = -1014,
        SocketOtherError = -1015,

        // 硬件错误
        JointRegistError = -2001,
        EepromRead = -2010,
        EepromWrite = -2011,
        LsEncoderOverspeed = -2012,
        LsEncoderFbError = -2013,
        MsSignalZError = -2014,
        ThreePhaseCurrent = -2015,
        TorqueSensorReadError = -2016,
        JointStateError = -2017,

        // 内部通信错误
        CommunicateError = -2101,
        LostHeartWithDianaRobotError = -2102,

        // 系统错误
        CallingConflictError = -2201,
        CollisionError = -2202,
        NotFollowPositionCmd = -2203,
        NotFollowTcpCmd = -2204,
        NotAllAtOpState = -2205,
        OutRangeFeedback = -2206,
        EmergencyStop = -2207,
        NoInitParameter = -2208,
        NotMatchLoad = -2209,
        CannotMoveWhileFreeDriving = -2210,
        CannotMoveWhileZeroSpaceFreeDriving = -2211,
        NotEnabled = -2212,
        RobotInVirtualWall = -2214,
        ConflictTaskRunning = -2215,
        OutOfPhysicalRangeFeedback = -2216,
        OutSoftRangeFeedback = -2217,
        ConveyorNotOnline = -2218,
        ConveyorIsTracked = -2219,
        ConveyorCannotTrack = -2220,
        InputOutOfExtremePositionRange = -2221,
        SlopoverVirtualWall = -2222,
        SlopoverReduceVirtualWall = -2223,

        // 算法错误
        PlanError = -2301,
        InterpolatePositionError = -2302,
        InterpolateTorqueError = -2303,
        SingularValueError = -2304,
        PlannerError = -2305,
        HomePositionError = -2306,
        Fatal = -2307,
        PosLimit = -2308,
        ForceLimit = -2309,
        SpeedLimit = -2310,
        AccLimit = -2311,
        JerkLimit = -2312,
        MotionLimit = -2313,
        IkTrack = -2314,
        IkGeneral = -2315,
        PlanInput = -2316,
        PlanMoveJ = -2317,
        PlanMoveL = -2318,
        PlanMoveC = -2319,
        PlanBlend = -2320,
        PlanSpeedJ = -2321,
        PlanSpeedL = -2322,
        PlanServoJ = -2323,
        PlanServoL = -2324,
        MoveUnknown = -2325,
        MoveUnplan = -2326,
        MoveInput = -2327,
        MoveInterp = -2328,
        PlanTranslation = -2329,
        PlanRotation = -2330,
        PlanJoints = -2331,
        UnmatchedJointsNumber = -2332,
        TcpCaliFutileWps = -2333,
        TcpCaliFitFail = -2334,
        DhCaliFitWfFail = -2335,
        DhCaliFitTfFail = -2336,
        DhCaliFitDhFail = -2337,
        DhCaliInitFail = -2338,
        SlfMovSingular = -2339,
        SlfMovFutile = -2340,
        SlfMovJntLim = -2341,
        SlfMovSpdLim = -2342,
        SlfMovFail = -2343,
        SlfMovFfcFail = -2344,
        LoadIdentInitFail = -2345,
        LoadIdentUfbFail = -2346,
        LoadIdentFitFail = -2347,
        LoadIdentNonLoaded = -2348,
        IkCalcFail = -2349,
        FkCalcFail = -2350,
        IdCalcFail = -2351,
        FdCalcFail = -2352,
        JacobianCalcFail = -2353,

        // API系统错误
        ParameterPointerEqualsNullptr = -2901,
        ParameterNanOrInf = -2902,
        EnterForceModeError = -2903,
        CannotSetVelocityPercentValue = -2904,
        InputOutOfPhysicalPositionRange = -2905,

        // 其他错误
        ResourceUnavailable = -3001,
        DumpLogTimeout = -3002,
        DumpLogFailed = -3003,
        ResetDhFailed = -3004,
        IllegalParameter = -3006,
    };


    inline bool operator<(const Result &lhs, const Result &rhs) {
        return static_cast<int>(lhs) < static_cast<int>(rhs);
    }

    inline bool operator<(const Result &lhs, const int &rhs) {
        return static_cast<int>(lhs) < rhs;
    }

    inline bool operator==(const Result &lhs, const int &rhs) {
        return static_cast<int>(lhs) == rhs;
    }

    inline bool operator>(const Result &lhs, const Result &rhs) {
        return static_cast<int>(lhs) > static_cast<int>(rhs);
    }

    inline bool operator>(const Result &lhs, const int &rhs) {
        return static_cast<int>(lhs) > rhs;
    }

    inline std::string to_string(Result r) {
#define RESULT_CASE(name) case Result::name: return #name
        switch (r) {
            RESULT_CASE(PlanFinished);
            RESULT_CASE(NoError);
            RESULT_CASE(WsastartFail);
            RESULT_CASE(CreateSocketFail);
            RESULT_CASE(BindPortFail);
            RESULT_CASE(SocketReadFail);
            RESULT_CASE(SocketTimeout);
            RESULT_CASE(RecvfromFail);
            RESULT_CASE(SendtoFail);
            RESULT_CASE(LostHeartbeat);
            RESULT_CASE(LostRobotState);
            RESULT_CASE(GetDhFailed);
            RESULT_CASE(ReleaseBrakeFailed);
            RESULT_CASE(HoldBrakeFailed);
            RESULT_CASE(IpAddressNotRegister);
            RESULT_CASE(RobotArmOverNumber);
            RESULT_CASE(SocketOtherError);
            RESULT_CASE(JointRegistError);
            RESULT_CASE(EepromRead);
            RESULT_CASE(EepromWrite);
            RESULT_CASE(LsEncoderOverspeed);
            RESULT_CASE(LsEncoderFbError);
            RESULT_CASE(MsSignalZError);
            RESULT_CASE(ThreePhaseCurrent);
            RESULT_CASE(TorqueSensorReadError);
            RESULT_CASE(JointStateError);
            RESULT_CASE(CommunicateError);
            RESULT_CASE(LostHeartWithDianaRobotError);
            RESULT_CASE(CallingConflictError);
            RESULT_CASE(CollisionError);
            RESULT_CASE(NotFollowPositionCmd);
            RESULT_CASE(NotFollowTcpCmd);
            RESULT_CASE(NotAllAtOpState);
            RESULT_CASE(OutRangeFeedback);
            RESULT_CASE(EmergencyStop);
            RESULT_CASE(NoInitParameter);
            RESULT_CASE(NotMatchLoad);
            RESULT_CASE(CannotMoveWhileFreeDriving);
            RESULT_CASE(CannotMoveWhileZeroSpaceFreeDriving);
            RESULT_CASE(NotEnabled);
            RESULT_CASE(RobotInVirtualWall);
            RESULT_CASE(ConflictTaskRunning);
            RESULT_CASE(OutOfPhysicalRangeFeedback);
            RESULT_CASE(OutSoftRangeFeedback);
            RESULT_CASE(ConveyorNotOnline);
            RESULT_CASE(ConveyorIsTracked);
            RESULT_CASE(ConveyorCannotTrack);
            RESULT_CASE(InputOutOfExtremePositionRange);
            RESULT_CASE(SlopoverVirtualWall);
            RESULT_CASE(SlopoverReduceVirtualWall);
            RESULT_CASE(PlanError);
            RESULT_CASE(InterpolatePositionError);
            RESULT_CASE(InterpolateTorqueError);
            RESULT_CASE(SingularValueError);
            RESULT_CASE(PlannerError);
            RESULT_CASE(HomePositionError);
            RESULT_CASE(Fatal);
            RESULT_CASE(PosLimit);
            RESULT_CASE(ForceLimit);
            RESULT_CASE(SpeedLimit);
            RESULT_CASE(AccLimit);
            RESULT_CASE(JerkLimit);
            RESULT_CASE(MotionLimit);
            RESULT_CASE(IkTrack);
            RESULT_CASE(IkGeneral);
            RESULT_CASE(PlanInput);
            RESULT_CASE(PlanMoveJ);
            RESULT_CASE(PlanMoveL);
            RESULT_CASE(PlanMoveC);
            RESULT_CASE(PlanBlend);
            RESULT_CASE(PlanSpeedJ);
            RESULT_CASE(PlanSpeedL);
            RESULT_CASE(PlanServoJ);
            RESULT_CASE(PlanServoL);
            RESULT_CASE(MoveUnknown);
            RESULT_CASE(MoveUnplan);
            RESULT_CASE(MoveInput);
            RESULT_CASE(MoveInterp);
            RESULT_CASE(PlanTranslation);
            RESULT_CASE(PlanRotation);
            RESULT_CASE(PlanJoints);
            RESULT_CASE(UnmatchedJointsNumber);
            RESULT_CASE(TcpCaliFutileWps);
            RESULT_CASE(TcpCaliFitFail);
            RESULT_CASE(DhCaliFitWfFail);
            RESULT_CASE(DhCaliFitTfFail);
            RESULT_CASE(DhCaliFitDhFail);
            RESULT_CASE(DhCaliInitFail);
            RESULT_CASE(SlfMovSingular);
            RESULT_CASE(SlfMovFutile);
            RESULT_CASE(SlfMovJntLim);
            RESULT_CASE(SlfMovSpdLim);
            RESULT_CASE(SlfMovFail);
            RESULT_CASE(SlfMovFfcFail);
            RESULT_CASE(LoadIdentInitFail);
            RESULT_CASE(LoadIdentUfbFail);
            RESULT_CASE(LoadIdentFitFail);
            RESULT_CASE(LoadIdentNonLoaded);
            RESULT_CASE(IkCalcFail);
            RESULT_CASE(FkCalcFail);
            RESULT_CASE(IdCalcFail);
            RESULT_CASE(FdCalcFail);
            RESULT_CASE(ParameterPointerEqualsNullptr);
            RESULT_CASE(ParameterNanOrInf);
            RESULT_CASE(EnterForceModeError);
            RESULT_CASE(CannotSetVelocityPercentValue);
            RESULT_CASE(InputOutOfPhysicalPositionRange);
            RESULT_CASE(ResourceUnavailable);
            RESULT_CASE(DumpLogTimeout);
            RESULT_CASE(DumpLogFailed);
            RESULT_CASE(ResetDhFailed);
            RESULT_CASE(IllegalParameter);
            default: return "Unknown(" + std::to_string(static_cast<int>(r)) + ")";
        }
#undef RESULT_CASE
    }

    inline std::ostream &operator<<(std::ostream &os, Result r) {
        return os << to_string(r);
    }
}
