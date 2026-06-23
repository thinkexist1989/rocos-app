#ifndef ROCOS_APP_MOTION_DIANA_ERROR_CODES_H
#define ROCOS_APP_MOTION_DIANA_ERROR_CODES_H

namespace rocos::motion {

enum class ErrorCode : int {
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


//TODO: MotionResultCode没有存在必要，直接用ErrorCode即可
enum class MotionResultCode {
    Ok,
    Busy,
    InvalidCommand,
    InvalidNumber,
    InvalidState,
    Unsupported,
    PlanningFailed,
    ExecutionFailed,
    SafetyViolation,
    HardwareFault
};

inline int toErrorCode(MotionResultCode code) {
    switch (code) {
        case MotionResultCode::Ok:
            return 0;
        case MotionResultCode::Busy:
            return static_cast<int>(ErrorCode::ConflictTaskRunning);
        case MotionResultCode::InvalidCommand:
            return static_cast<int>(ErrorCode::IllegalParameter);
        case MotionResultCode::InvalidNumber:
            return static_cast<int>(ErrorCode::ParameterNanOrInf);
        case MotionResultCode::InvalidState:
            return static_cast<int>(ErrorCode::NotAllAtOpState);
        case MotionResultCode::Unsupported:
            return static_cast<int>(ErrorCode::CallingConflictError);
        case MotionResultCode::PlanningFailed:
            return static_cast<int>(ErrorCode::PlanError);
        case MotionResultCode::ExecutionFailed:
            return static_cast<int>(ErrorCode::MoveInterp);
        case MotionResultCode::SafetyViolation:
            return static_cast<int>(ErrorCode::Fatal);
        case MotionResultCode::HardwareFault:
            return static_cast<int>(ErrorCode::JointRegistError);
    }

    return static_cast<int>(ErrorCode::IllegalParameter);
}

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_DIANA_ERROR_CODES_H
