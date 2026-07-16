#include "lua_script_engine.hpp"

#include <sol/sol.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <fstream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>
#include <tuple>
#include <utility>

#include "logger.hpp"
#include "robot.hpp"

namespace rocos {
namespace {

constexpr const char* SCRIPT_STOP_MARKER = "__ROCOS_SCRIPT_STOPPED__";
constexpr auto MOTION_POLL_INTERVAL = std::chrono::milliseconds(2);

std::string normalizeFilename(const std::string& filename) {
    return std::filesystem::path(filename).lexically_normal().generic_string();
}

bool pathStartsWith(const std::filesystem::path& path,
                    const std::filesystem::path& root) {
    auto path_it_ = path.begin();
    auto root_it_ = root.begin();
    for (; root_it_ != root.end(); ++root_it_, ++path_it_) {
        if (path_it_ == path.end() || *path_it_ != *root_it_) {
            return false;
        }
    }
    return true;
}

}  // namespace

struct LuaScriptEngine::Impl {
    struct Breakpoint {
        std::string filename;
        int line{0};

        bool operator<(const Breakpoint& rhs) const noexcept {
            return std::tie(filename, line) <
                   std::tie(rhs.filename, rhs.line);
        }
    };

    struct Pose {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double qx{0.0};
        double qy{0.0};
        double qz{0.0};
        double qw{1.0};
    };

    Impl(Robot& robot, std::filesystem::path scripts_root)
        : robot_(robot),
          scripts_root_(std::filesystem::absolute(
              std::move(scripts_root)).lexically_normal()),
          log_ptr_(Logger::getInstance("LuaScriptEngine")) {}

    ~Impl() {
        Stop();
        joinWorker();
    }

    Result LoadSource(const std::string& source, const std::string& filename) {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        if (source.empty() || filename.empty()) {
            return Result::IllegalParameter;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Running ||
                state_ == State::Pausing ||
                state_ == State::Paused ||
                state_ == State::Stopping) {
                return Result::LuaStateConflict;
            }
        }
        joinWorker();

        std::lock_guard<std::mutex> lock(mutex_);
        initializeLua();
        filename_ = normalizeFilename(filename);
        source_ = source;
        location_ = SourceLocation{filename_, 0};
        error_.clear();
        stop_requested_ = false;
        pause_requested_ = false;
        step_mode_ = false;
        step_started_ = false;
        motion_active_ = false;
        stop_command_sent_ = false;

        sol::load_result load_result_ =
            lua_->load(source_, "@" + filename_);
        if (!load_result_.valid()) {
            const sol::error error_ = load_result_;
            state_ = State::Failed;
            this->error_ = error_.what();
            log_ptr_->error("Lua 脚本编译失败: {}", this->error_);
            return Result::LuaExecutionError;
        }

        function_ = load_result_;
        ++script_counter_;
        script_id_ = "script_" + std::to_string(script_counter_);
        state_ = State::Loaded;
        return Result::NoError;
    }

    Result LoadFile(const std::filesystem::path& relative_path) {
        if (relative_path.empty() || relative_path.is_absolute()) {
            return Result::LuaFileError;
        }

        std::error_code error_code_;
        const auto root_ =
            std::filesystem::weakly_canonical(scripts_root_, error_code_);
        if (error_code_) {
            return Result::LuaFileError;
        }

        const auto script_path_ =
            std::filesystem::canonical(root_ / relative_path, error_code_);
        if (error_code_ || !pathStartsWith(script_path_, root_)) {
            return Result::LuaFileError;
        }

        std::ifstream stream_(script_path_);
        if (!stream_.is_open()) {
            log_ptr_->error("无法打开 Lua 脚本: {}", script_path_.string());
            return Result::LuaFileError;
        }

        std::ostringstream buffer_;
        buffer_ << stream_.rdbuf();
        if (stream_.bad()) {
            log_ptr_->error("读取 Lua 脚本失败: {}", script_path_.string());
            return Result::LuaFileError;
        }

        const auto display_path_ =
            std::filesystem::relative(script_path_, root_, error_code_);
        if (error_code_) {
            return Result::LuaFileError;
        }
        return LoadSource(buffer_.str(), display_path_.generic_string());
    }

    Result Run() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Loaded) {
                return Result::LuaStateConflict;
            }
            state_ = State::Running;
            stop_requested_ = false;
            pause_requested_ = false;
            step_mode_ = false;
            step_started_ = false;
        }

        try {
            worker_ = std::thread(&Impl::execute, this);
        } catch (const std::system_error& exception_) {
            std::lock_guard<std::mutex> lock(mutex_);
            state_ = State::Loaded;
            error_ = exception_.what();
            log_ptr_->error("启动 Lua worker 失败: {}", this->error_);
            return Result::ResourceUnavailable;
        }
        return Result::NoError;
    }

    Result Pause() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Running) {
                return Result::LuaStateConflict;
            }
            pause_requested_ = true;
            state_ = State::Pausing;
            motion_active_ = this->motion_active_;
        }

        if (motion_active_) {
            const Result result_ = robot_.PauseMotion();
            if (result_ != Result::NoError) {
                std::lock_guard<std::mutex> lock(mutex_);
                if (state_ == State::Pausing) {
                    pause_requested_ = false;
                    state_ = State::Running;
                }
                return result_;
            }
        }
        return Result::NoError;
    }

    Result Resume() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Paused) {
                return Result::LuaStateConflict;
            }
            motion_active_ = this->motion_active_;
        }

        if (motion_active_) {
            const Result result_ = robot_.ResumeMotion();
            if (result_ != Result::NoError) {
                return result_;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pause_requested_ = false;
            step_mode_ = false;
            step_started_ = false;
            state_ = State::Running;
        }
        control_cv_.notify_all();
        return Result::NoError;
    }

    Result Stop() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_;
        bool pause_requested_;
        State previous_state_;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Empty ||
                state_ == State::Completed ||
                state_ == State::Failed ||
                state_ == State::Stopped) {
                return Result::NoError;
            }
            if (state_ == State::Loaded) {
                state_ = State::Stopped;
                return Result::NoError;
            }
            motion_active_ = this->motion_active_;
            pause_requested_ = this->pause_requested_;
            previous_state_ = state_;
            state_ = State::Stopping;
            stop_requested_ = true;
            stop_command_sent_ = motion_active_;
            this->pause_requested_ = false;
        }
        control_cv_.notify_all();

        if (motion_active_) {
            const Result result_ = robot_.StopMotion();
            if (result_ != Result::NoError &&
                robot_.GetStateString() != "STOPPED") {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = previous_state_;
                stop_requested_ = false;
                stop_command_sent_ = false;
                this->pause_requested_ = pause_requested_;
                return result_;
            }
        }
        return Result::NoError;
    }

    Result Step() {
        std::lock_guard<std::mutex> lifecycle_lock_(lifecycle_mutex_);
        bool motion_active_ = false;
        bool start_worker_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ == State::Loaded) {
                state_ = State::Running;
                stop_requested_ = false;
                pause_requested_ = false;
                step_mode_ = true;
                step_started_ = false;
                start_worker_ = true;
            } else {
                if (state_ != State::Paused) {
                    return Result::LuaStateConflict;
                }
                motion_active_ = this->motion_active_;
            }
        }

        if (start_worker_) {
            try {
                worker_ = std::thread(&Impl::execute, this);
            } catch (const std::system_error& exception_) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = State::Loaded;
                step_mode_ = false;
                error_ = exception_.what();
                log_ptr_->error(
                    "启动 Lua worker 失败: {}", this->error_);
                return Result::ResourceUnavailable;
            }
            return Result::NoError;
        }

        if (motion_active_) {
            const Result result_ = robot_.ResumeMotion();
            if (result_ != Result::NoError) {
                return result_;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (state_ != State::Paused) {
                return Result::LuaStateConflict;
            }
            pause_requested_ = false;
            step_mode_ = true;
            step_started_ = true;
            state_ = State::Running;
        }
        control_cv_.notify_all();
        return Result::NoError;
    }

    Result AddBreakpoint(const std::string& filename, int line) {
        if (line <= 0) {
            return Result::LuaInvalidBreakpoint;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const std::string normalized_ =
            normalizeFilename(filename.empty() ? filename_ : filename);
        if (normalized_.empty()) {
            return Result::LuaInvalidBreakpoint;
        }
        breakpoints_.insert(Breakpoint{normalized_, line});
        return Result::NoError;
    }

    Result RemoveBreakpoint(const std::string& filename, int line) {
        if (line <= 0) {
            return Result::LuaInvalidBreakpoint;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const std::string normalized_ =
            normalizeFilename(filename.empty() ? filename_ : filename);
        if (breakpoints_.erase(Breakpoint{normalized_, line}) == 0) {
            return Result::LuaInvalidBreakpoint;
        }
        return Result::NoError;
    }

    Result ClearBreakpoints() {
        std::lock_guard<std::mutex> lock(mutex_);
        breakpoints_.clear();
        return Result::NoError;
    }

    Status GetStatus() const {
        std::lock_guard<std::mutex> lock(mutex_);
        Status status_;
        status_.state = state_;
        status_.script_id = script_id_;
        status_.location = location_;
        status_.error = error_;
        status_.motion_active = motion_active_;
        status_.breakpoints.reserve(breakpoints_.size());
        for (const auto& breakpoint_ : breakpoints_) {
            status_.breakpoints.push_back(
                SourceLocation{breakpoint_.filename, breakpoint_.line});
        }
        return status_;
    }

private:
    void initializeLua() {
        function_ = sol::protected_function();
        lua_ = std::make_unique<sol::state>();
        lua_->open_libraries(sol::lib::base,
                             sol::lib::math,
                             sol::lib::string,
                             sol::lib::table,
                             sol::lib::coroutine,
                             sol::lib::utf8);

        (*lua_)["dofile"] = sol::nil;
        (*lua_)["loadfile"] = sol::nil;
        (*lua_)["require"] = sol::nil;

        *static_cast<Impl**>(lua_getextraspace(lua_->lua_state())) = this;
        registerBindings();
    }

    void registerBindings() {
        sol::table robot_table_ = lua_->create_named_table("robot");

        robot_table_.set_function(
            "MoveJ",
            [this](const sol::table& joints,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                const int joint_count_ = robot_.getJointNum();
                if (joint_count_ <= 0 ||
                    joints.size() != static_cast<std::size_t>(joint_count_)) {
                    throw sol::error("MoveJ 关节数量与机器人不匹配");
                }

                JntArray target_(static_cast<unsigned int>(joint_count_));
                for (int index_ = 0; index_ < joint_count_; ++index_) {
                    const sol::object value_ = joints[index_ + 1];
                    if (!value_.is<double>()) {
                        throw sol::error("MoveJ 关节值必须为 number");
                    }
                    target_(static_cast<unsigned int>(index_)) =
                        requireFinite(value_.as<double>(), "MoveJ joint");
                }

                executeMotion(robot_.MoveJ(
                    target_,
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveL",
            [this](const sol::table& pose,
                   sol::optional<std::string> tool_name,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveL(
                    toFrame(pose),
                    tool_name.value_or(""),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveCByCenter",
            [this](const sol::table& start_pose,
                   const sol::table& center_pose,
                   double theta,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveC(
                    toFrame(start_pose),
                    toFrame(center_pose),
                    requireFinite(theta, "theta"),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "MoveCByPoints",
            [this](const sol::table& start_pose,
                   const sol::table& via_pose,
                   const sol::table& goal_pose,
                   sol::optional<double> velocity,
                   sol::optional<double> acceleration,
                   sol::optional<double> jerk) {
                executeMotion(robot_.MoveC(
                    toFrame(start_pose),
                    toFrame(via_pose),
                    toFrame(goal_pose),
                    requirePositive(velocity.value_or(1.0), "velocity"),
                    requirePositive(acceleration.value_or(2.0), "acceleration"),
                    requirePositive(jerk.value_or(10.0), "jerk")));
            });

        robot_table_.set_function(
            "GetState", [this]() { return robot_.GetStateString(); });
        robot_table_.set_function(
            "IsEnabled", [this]() { return robot_.IsEnabled(); });
        robot_table_.set_function(
            "GetJointCount", [this]() { return robot_.getJointNum(); });
        robot_table_.set_function(
            "GetJointPosition", [this](int index) {
                const int joint_count_ = robot_.getJointNum();
                if (index <= 0 || index > joint_count_) {
                    throw sol::error("joint index 超出范围");
                }
                return robot_.getJointPosition(index - 1);
            });
        robot_table_.set_function(
            "Sleep", [this](int milliseconds) { sleepInterruptibly(milliseconds); });
    }

    static void debugHook(lua_State* lua_state, lua_Debug* debug_info) {
        auto* impl_ =
            *static_cast<Impl**>(lua_getextraspace(lua_state));
        if (impl_ != nullptr) {
            impl_->handleHook(lua_state, debug_info);
        }
    }

    void handleHook(lua_State* lua_state, lua_Debug* debug_info) {
        bool should_stop_ = false;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (debug_info->event == LUA_HOOKLINE) {
                location_ = SourceLocation{filename_, debug_info->currentline};

                if (step_mode_) {
                    if (step_started_) {
                        step_mode_ = false;
                        pause_requested_ = true;
                    } else {
                        step_started_ = true;
                    }
                }

                if (breakpoints_.count(
                        Breakpoint{filename_, debug_info->currentline}) != 0) {
                    pause_requested_ = true;
                }
            }

            if (pause_requested_ && !stop_requested_) {
                state_ = State::Paused;
                control_cv_.wait(lock, [this]() {
                    return !pause_requested_ || stop_requested_;
                });
                if (!stop_requested_) {
                    state_ = State::Running;
                }
            }
            should_stop_ = stop_requested_;
        }

        if (should_stop_) {
            luaL_error(lua_state, "%s", SCRIPT_STOP_MARKER);
        }
    }

    void execute() {
        lua_State* lua_state_ = lua_->lua_state();
        lua_sethook(lua_state_, &Impl::debugHook,
                    LUA_MASKLINE | LUA_MASKCOUNT, 1000);

        try {
            sol::protected_function_result result_ = function_();
            lua_sethook(lua_state_, nullptr, 0, 0);

            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = false;
            if (!result_.valid()) {
                const sol::error error_ = result_;
                this->error_ = error_.what();
                if (stop_requested_ &&
                    this->error_.find(SCRIPT_STOP_MARKER) !=
                        std::string::npos) {
                    state_ = State::Stopped;
                    this->error_.clear();
                } else {
                    state_ = State::Failed;
                }
            } else if (stop_requested_) {
                state_ = State::Stopped;
                error_.clear();
            } else {
                state_ = State::Completed;
                error_.clear();
            }
        } catch (const std::exception& exception_) {
            lua_sethook(lua_state_, nullptr, 0, 0);
            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = false;
            if (stop_requested_ &&
                std::string(exception_.what()).find(SCRIPT_STOP_MARKER) !=
                    std::string::npos) {
                state_ = State::Stopped;
                error_.clear();
            } else {
                state_ = State::Failed;
                error_ = exception_.what();
                log_ptr_->error("Lua 脚本执行失败: {}", error_);
            }
        }
        control_cv_.notify_all();
    }

    void executeMotion(Result result) {
        if (result != Result::NoError && result != Result::PlanFinished) {
            throw sol::error(
                "机器人运动提交失败: " + to_string(result));
        }
        if (result == Result::PlanFinished) {
            return;
        }

        bool pause_requested_ = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_active_ = true;
            pause_requested_ = this->pause_requested_;
        }

        if (pause_requested_) {
            const Result pause_result_ = robot_.PauseMotion();
            if (pause_result_ != Result::NoError) {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_active_ = false;
                throw sol::error(
                    "暂停机器人运动失败: " + to_string(pause_result_));
            }
        }

        for (;;) {
            bool should_stop_ = false;
            bool stop_command_sent_ = false;
            bool should_pause_ = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                should_stop_ = stop_requested_;
                stop_command_sent_ = this->stop_command_sent_;
                should_pause_ = pause_requested_;
            }

            if (should_stop_ && !stop_command_sent_) {
                const Result stop_result_ = robot_.StopMotion();
                const bool already_stopped_ =
                    robot_.GetStateString() == "STOPPED";
                if (stop_result_ != Result::NoError && !already_stopped_) {
                    std::lock_guard<std::mutex> lock(mutex_);
                    stop_requested_ = false;
                    motion_active_ = false;
                    throw sol::error(
                        "停止机器人运动失败: " + to_string(stop_result_));
                }
                std::lock_guard<std::mutex> lock(mutex_);
                this->stop_command_sent_ = true;
            }

            const std::string robot_state_ = robot_.GetStateString();
            if (robot_state_ == "STOPPED") {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_active_ = false;
                if (should_stop_) {
                    throw sol::error(SCRIPT_STOP_MARKER);
                }
                return;
            }
            if (robot_state_ == "PAUSED" && should_pause_) {
                std::lock_guard<std::mutex> lock(mutex_);
                state_ = State::Paused;
            }
            if (robot_state_ == "ERROR_STATE" ||
                robot_state_ == "IDLE" ||
                robot_state_ == "UNKNOWN_STATE") {
                std::lock_guard<std::mutex> lock(mutex_);
                motion_active_ = false;
                throw sol::error(
                    "机器人运动异常终止，状态: " + robot_state_);
            }

            std::unique_lock<std::mutex> lock(mutex_);
            control_cv_.wait_for(lock, MOTION_POLL_INTERVAL);
        }
    }

    void sleepInterruptibly(int milliseconds) {
        if (milliseconds < 0) {
            throw sol::error("Sleep 时长不能为负数");
        }

        auto remaining_ = std::chrono::milliseconds(milliseconds);
        while (remaining_.count() > 0) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (stop_requested_) {
                throw sol::error(SCRIPT_STOP_MARKER);
            }
            if (pause_requested_) {
                state_ = State::Paused;
                control_cv_.wait(lock, [this]() {
                    return !pause_requested_ || stop_requested_;
                });
                if (stop_requested_) {
                    throw sol::error(SCRIPT_STOP_MARKER);
                }
                state_ = State::Running;
            }

            const auto start_ = std::chrono::steady_clock::now();
            control_cv_.wait_for(lock, remaining_, [this]() {
                return stop_requested_ || pause_requested_;
            });
            const auto elapsed_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_);
            remaining_ = elapsed_ >= remaining_
                             ? std::chrono::milliseconds(0)
                             : remaining_ - elapsed_;
        }
    }

    static double requireFinite(double value, const char* name) {
        if (!std::isfinite(value)) {
            throw sol::error(std::string(name) + " 包含 NaN 或 Inf");
        }
        return value;
    }

    static double requirePositive(double value, const char* name) {
        requireFinite(value, name);
        if (value <= 0.0) {
            throw sol::error(std::string(name) + " 必须大于 0");
        }
        return value;
    }

    static double tableNumber(const sol::table& table, const char* key) {
        const sol::object value_ = table[key];
        if (!value_.is<double>()) {
            throw sol::error(std::string("pose.") + key + " 必须为 number");
        }
        return requireFinite(value_.as<double>(), key);
    }

    static Frame toFrame(const sol::table& table) {
        Pose pose_;
        pose_.x = tableNumber(table, "x");
        pose_.y = tableNumber(table, "y");
        pose_.z = tableNumber(table, "z");
        pose_.qx = tableNumber(table, "qx");
        pose_.qy = tableNumber(table, "qy");
        pose_.qz = tableNumber(table, "qz");
        pose_.qw = tableNumber(table, "qw");

        const double norm_ = std::sqrt(
            pose_.qx * pose_.qx + pose_.qy * pose_.qy +
            pose_.qz * pose_.qz + pose_.qw * pose_.qw);
        if (norm_ <= 1e-12) {
            throw sol::error("pose quaternion 不能为零");
        }

        return Frame(
            KDL::Rotation::Quaternion(
                pose_.qx / norm_,
                pose_.qy / norm_,
                pose_.qz / norm_,
                pose_.qw / norm_),
            KDL::Vector(pose_.x, pose_.y, pose_.z));
    }

    void joinWorker() {
        if (worker_.joinable()) {
            worker_.join();
        }
    }

    Robot& robot_;
    std::filesystem::path scripts_root_;
    Logger::logger_ptr log_ptr_;

    std::mutex lifecycle_mutex_;
    mutable std::mutex mutex_;
    std::condition_variable control_cv_;
    std::thread worker_;
    std::unique_ptr<sol::state> lua_;
    sol::protected_function function_;

    State state_{State::Empty};
    std::string source_;
    std::string filename_;
    std::string script_id_;
    std::string error_;
    SourceLocation location_;
    std::set<Breakpoint> breakpoints_;
    std::uint64_t script_counter_{0};
    bool stop_requested_{false};
    bool pause_requested_{false};
    bool step_mode_{false};
    bool step_started_{false};
    bool motion_active_{false};
    bool stop_command_sent_{false};
};

LuaScriptEngine::LuaScriptEngine(
    Robot& robot, std::filesystem::path scripts_root)
    : impl_(std::make_unique<Impl>(
          robot, std::move(scripts_root))) {}

LuaScriptEngine::~LuaScriptEngine() = default;

Result LuaScriptEngine::LoadSource(
    const std::string& source, const std::string& filename) {
    return impl_->LoadSource(source, filename);
}

Result LuaScriptEngine::LoadFile(
    const std::filesystem::path& relative_path) {
    return impl_->LoadFile(relative_path);
}

Result LuaScriptEngine::Run() {
    return impl_->Run();
}

Result LuaScriptEngine::Pause() {
    return impl_->Pause();
}

Result LuaScriptEngine::Resume() {
    return impl_->Resume();
}

Result LuaScriptEngine::Stop() {
    return impl_->Stop();
}

Result LuaScriptEngine::Step() {
    return impl_->Step();
}

Result LuaScriptEngine::AddBreakpoint(
    const std::string& filename, int line) {
    return impl_->AddBreakpoint(filename, line);
}

Result LuaScriptEngine::RemoveBreakpoint(
    const std::string& filename, int line) {
    return impl_->RemoveBreakpoint(filename, line);
}

Result LuaScriptEngine::ClearBreakpoints() {
    return impl_->ClearBreakpoints();
}

LuaScriptEngine::Status LuaScriptEngine::GetStatus() const {
    return impl_->GetStatus();
}

const char* LuaScriptEngine::ToString(State state) noexcept {
    switch (state) {
        case State::Empty:
            return "EMPTY";
        case State::Loaded:
            return "LOADED";
        case State::Running:
            return "RUNNING";
        case State::Pausing:
            return "PAUSING";
        case State::Paused:
            return "PAUSED";
        case State::Stopping:
            return "STOPPING";
        case State::Completed:
            return "COMPLETED";
        case State::Failed:
            return "FAILED";
        case State::Stopped:
            return "STOPPED";
    }
    return "UNKNOWN";
}

}  // namespace rocos
