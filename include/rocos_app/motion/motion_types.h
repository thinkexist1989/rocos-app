#ifndef ROCOS_APP_MOTION_MOTION_TYPES_H
#define ROCOS_APP_MOTION_MOTION_TYPES_H

#include <rocos_app/motion/diana_error_codes.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace rocos::motion {

enum class ReferenceSpace : std::uint8_t {
    None = 0,
    Joint = 1,
    Cartesian = 2,
    Both = 3
};

inline bool hasReferenceSpace(ReferenceSpace value,
                              ReferenceSpace expected) {
    return (static_cast<std::uint8_t>(value) &
            static_cast<std::uint8_t>(expected)) != 0;
}

enum class MotionTaskStatus {
    None,
    Accepted,
    Running,
    Paused,
    Stopping,
    Finished,
    Failed,
    Cancelled
};

enum class MotionStepStatus {
    Running,
    Paused,
    Finished,
    Stopped,
    Failed
};

struct MotionResult {
    bool success{false};
    MotionResultCode result{MotionResultCode::Ok};
    int api_error_code{0};
    std::string message;

    static MotionResult ok(std::string text = "ok") {
        return MotionResult{true, MotionResultCode::Ok, 0, std::move(text)};
    }

    static MotionResult fail(MotionResultCode code, std::string text) {
        return MotionResult{false, code, toDianaErrorCode(code), std::move(text)};
    }

    static MotionResult failWithApiCode(MotionResultCode code,
                                        int api_code,
                                        std::string text) {
        return MotionResult{false, code, api_code, std::move(text)};
    }
};

struct JointReference {
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
};

struct MotionReference {
    ReferenceSpace space{ReferenceSpace::None};
    JointReference joint;
};

struct MotionStepResult {
    MotionStepStatus status{MotionStepStatus::Running};
    MotionResult result{MotionResult::ok()};
    std::optional<MotionReference> reference;
};

}  // namespace rocos::motion

#endif  // ROCOS_APP_MOTION_MOTION_TYPES_H
