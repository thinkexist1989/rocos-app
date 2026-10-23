#include "frame_registry.hpp"
#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <sys/stat.h>
#include <unistd.h>

namespace rocos {
FrameRegistry::FrameRegistry(std::string path)
    : path_(std::move(path)), state_(std::make_shared<const FrameRegistryState>()) {}

bool FrameRegistry::validName(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    for (const unsigned char ch : name)
        if (!((ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
              (ch >= '0' && ch <= '9') || ch == '_' || ch == '-' || ch == '.')) return false;
    return true;
}
bool FrameRegistry::validFrame(const KDL::Frame& frame) {
    for (int i = 0; i < 3; ++i) {
        if (!std::isfinite(frame.p[i])) return false;
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(frame.M(i, j))) return false;
            double dot = 0;
            for (int k = 0; k < 3; ++k) dot += frame.M(k, i) * frame.M(k, j);
            if (std::abs(dot - (i == j ? 1 : 0)) > 1e-6) return false;
        }
    }
    return KDL::dot(frame.M.UnitX() * frame.M.UnitY(), frame.M.UnitZ()) > 1 - 1e-6;
}
std::shared_ptr<const FrameRegistryState> FrameRegistry::snapshot() const {
    return std::atomic_load(&state_);
}
Result FrameRegistry::resolveTcp(const FrameRegistryState& state, const KDL::Frame& flange,
                                 KDL::Frame& tcp, const std::string& tool_name) {
    if (!validFrame(flange)) return Result::FkCalcFail;
    const auto& name = tool_name.empty() ? state.active_tool : tool_name;
    if (name.empty()) { tcp = flange; return Result::NoError; }
    if (!validName(name)) return Result::IllegalParameter;
    const auto found = state.tools.find(name);
    if (found == state.tools.end()) return Result::ResourceUnavailable;
    if (!validFrame(found->second)) return Result::IllegalParameter;
    const auto result = flange * found->second;
    if (!validFrame(result)) return Result::FkCalcFail;
    tcp = result;
    return Result::NoError;
}
std::vector<std::string> FrameRegistry::names(bool tool) const {
    auto state = snapshot();
    const auto& frames = tool ? state->tools : state->objects;
    std::vector<std::string> names;
    for (const auto& item : frames) names.push_back(item.first);
    return names;
}
Result FrameRegistry::get(const std::string& name, bool tool, KDL::Frame& frame) const {
    auto state = snapshot();
    const auto& frames = tool ? state->tools : state->objects;
    auto found = frames.find(name);
    if (found == frames.end()) return Result::ResourceUnavailable;
    frame = found->second;
    return Result::NoError;
}
Result FrameRegistry::publish(std::shared_ptr<FrameRegistryState> next, bool persist) {
    if (persist) {
        const auto result = write(*next, path_);
        if (result != Result::NoError) return result;
    }
    std::atomic_store(&state_, std::shared_ptr<const FrameRegistryState>(std::move(next)));
    return Result::NoError;
}
Result FrameRegistry::set(const std::string& name, const KDL::Frame& frame, bool tool,
                          bool add_only, bool persist, const std::string& previous_name) {
    if (!validName(name) || !validFrame(frame)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    auto& frames = tool ? next->tools : next->objects;
    if (add_only && frames.count(name)) return Result::CallingConflictError;
    if (!previous_name.empty()) {
        if (!validName(previous_name)) return Result::IllegalParameter;
        if (!frames.count(previous_name)) return Result::ResourceUnavailable;
        if (previous_name != name) {
            if (frames.count(name)) return Result::CallingConflictError;
            frames.erase(previous_name);
            auto& active = tool ? next->active_tool : next->active_object;
            if (active == previous_name) active = name;
        }
    }
    frames[name] = frame;
    return publish(std::move(next), persist);
}
Result FrameRegistry::remove(const std::string& name, bool tool, bool persist) {
    if (!validName(name)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    auto& frames = tool ? next->tools : next->objects;
    auto& active = tool ? next->active_tool : next->active_object;
    if (!frames.erase(name)) return Result::ResourceUnavailable;
    if (active == name) active.clear();
    return publish(std::move(next), persist);
}
Result FrameRegistry::activate(const std::string& name, bool tool, bool persist) {
    if (!name.empty() && !validName(name)) return Result::IllegalParameter;
    std::lock_guard<std::mutex> lock(writer_mutex_);
    auto next = std::make_shared<FrameRegistryState>(*snapshot());
    const auto& frames = tool ? next->tools : next->objects;
    if (!name.empty() && !frames.count(name)) return Result::ResourceUnavailable;
    (tool ? next->active_tool : next->active_object) = name;
    return publish(std::move(next), persist);
}
Result FrameRegistry::write(const FrameRegistryState& state, const std::string& path) {
    if (path.empty()) return Result::IllegalParameter;
    std::string temporary = path + ".tmp.XXXXXX";
    int fd = -1;
    FILE* file = nullptr;
    try {
        YAML::Node root;
        root["active_tool"] = state.active_tool;
        root["active_object"] = state.active_object;
        for (bool tool : {true, false}) {
            const auto key = tool ? "tools" : "objects";
            root[key] = YAML::Node(YAML::NodeType::Map);
            for (const auto& item : tool ? state.tools : state.objects) {
                auto node = root[key][item.first];
                for (int i = 0; i < 3; ++i) node["position"].push_back(item.second.p[i]);
                double x, y, z, w;
                item.second.M.GetQuaternion(x, y, z, w);
                for (double value : {x, y, z, w}) node["orientation"].push_back(value);
            }
        }
        YAML::Emitter emitter;
        emitter.SetDoublePrecision(17);
        emitter << root;
        if (!emitter.good()) return Result::ResourceUnavailable;
        fd = mkstemp(temporary.data());
        if (fd < 0) return Result::ResourceUnavailable;
        // 原文件只有完整的新快照写好后才会被原子替换。
        struct stat existing{};
        if (::stat(path.c_str(), &existing) == 0) fchmod(fd, existing.st_mode & 0777);
        else fchmod(fd, 0640);
        file = fdopen(fd, "w");
        if (!file) { ::close(fd); std::remove(temporary.c_str()); return Result::ResourceUnavailable; }
        bool ok = fwrite(emitter.c_str(), 1, emitter.size(), file) == emitter.size();
        ok = fflush(file) == 0 && ok;
        ok = fsync(fd) == 0 && ok;
        ok = fclose(file) == 0 && ok;
        file = nullptr;
        if (ok && std::rename(temporary.c_str(), path.c_str()) == 0) return Result::NoError;
    } catch (const std::exception&) {
        if (file) fclose(file);
    }
    std::remove(temporary.c_str());
    return Result::ResourceUnavailable;
}
Result FrameRegistry::save(const std::string& path) const {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    return write(*snapshot(), path.empty() ? path_ : path);
}
Result FrameRegistry::load(const std::string& path) {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    try {
        const auto root = YAML::LoadFile(path.empty() ? path_ : path);
        if (!root.IsMap()) return Result::IllegalParameter;
        auto next = std::make_shared<FrameRegistryState>();
        for (bool tool : {true, false}) {
            const auto node = root[tool ? "tools" : "objects"];
            if (!node || node.IsNull()) continue;
            if (!node.IsMap()) return Result::IllegalParameter;
            auto& frames = tool ? next->tools : next->objects;
            for (const auto& item : node) {
                const auto name = item.first.as<std::string>();
                if (!validName(name)) return Result::IllegalParameter;
                const auto pos = item.second["position"], q = item.second["orientation"];
                if (!pos.IsSequence() || pos.size() != 3 || !q.IsSequence() || q.size() != 4) return Result::IllegalParameter;
                double x = q[0].as<double>(), y = q[1].as<double>(), z = q[2].as<double>(), w = q[3].as<double>();
                const double norm = std::sqrt(x*x + y*y + z*z + w*w);
                if (!std::isfinite(norm) || norm < 1e-12) return Result::IllegalParameter;
                KDL::Frame frame(KDL::Rotation::Quaternion(x/norm, y/norm, z/norm, w/norm),
                    KDL::Vector(pos[0].as<double>(), pos[1].as<double>(), pos[2].as<double>()));
                if (!validFrame(frame) || frames.count(name)) return Result::IllegalParameter;
                frames.emplace(name, frame);
            }
        }
        next->active_tool = root["active_tool"].as<std::string>("");
        next->active_object = root["active_object"].as<std::string>("");
        if ((!next->active_tool.empty() && !next->tools.count(next->active_tool)) ||
            (!next->active_object.empty() && !next->objects.count(next->active_object))) return Result::IllegalParameter;
        return publish(std::move(next), false);
    } catch (const std::exception&) { return Result::ResourceUnavailable; }
}
} // namespace rocos
