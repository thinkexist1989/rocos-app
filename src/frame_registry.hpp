#pragma once

#include "result.hpp"
#include <kdl/frames.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace rocos {
struct FrameRegistryState {
    std::map<std::string, KDL::Frame> tools, objects;
    std::string active_tool, active_object;
};

// 读者持有不可变快照；文件操作只阻塞写者，不占用读者的锁。
class FrameRegistry {
public:
    explicit FrameRegistry(std::string path = "frames.yaml");
    static bool validName(const std::string& name);
    static bool validFrame(const KDL::Frame& frame);
    // 同一注册表快照下解析工具并计算基座到 TCP；空名称使用活动工具，无工具时为法兰。
    static Result resolveTcp(const FrameRegistryState& state, const KDL::Frame& flange,
                             KDL::Frame& tcp, const std::string& tool_name = "");
    std::shared_ptr<const FrameRegistryState> snapshot() const;
    std::vector<std::string> names(bool tool) const;
    Result get(const std::string& name, bool tool, KDL::Frame& frame) const;
    Result set(const std::string& name, const KDL::Frame& frame, bool tool,
               bool add_only = false, bool persist = false, const std::string& previous_name = "");
    Result remove(const std::string& name, bool tool, bool persist = false);
    Result activate(const std::string& name, bool tool, bool persist = false);
    Result load(const std::string& path = "");
    Result save(const std::string& path = "") const;
    const std::string& path() const { return path_; }
private:
    Result publish(std::shared_ptr<FrameRegistryState> next, bool persist);
    static Result write(const FrameRegistryState& state, const std::string& path);
    const std::string path_;
    std::shared_ptr<const FrameRegistryState> state_;
    mutable std::mutex writer_mutex_;
};
} // namespace rocos
