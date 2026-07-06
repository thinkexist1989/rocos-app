//
// Created by think on 6/30/26.
//

#include "hardware.hpp"

#include <yaml-cpp/yaml.h>

#include <stdexcept>

#include "logger.hpp"

namespace rocos {

// ==========================================================================
// 辅助函数：YAML 字段安全读取
// ==========================================================================

/// @brief 安全读取 YAML 节点的 string 字段，不存在时返回默认值
static std::string yamlGetStr(const YAML::Node& node,
                              const std::string& key,
                              const std::string& default_val = "") {
    if (node[key]) {
        return node[key].as<std::string>();
    }
    return default_val;
}

/// @brief 安全读取 YAML 节点的 double 字段
static double yamlGetDouble(const YAML::Node& node,
                            const std::string& key,
                            double default_val = 0.0) {
    if (node[key]) {
        return node[key].as<double>();
    }
    return default_val;
}

/// @brief 安全读取 YAML 节点的 int 字段
static int yamlGetInt(const YAML::Node& node,
                      const std::string& key,
                      int default_val = 0) {
    if (node[key]) {
        return node[key].as<int>();
    }
    return default_val;
}

// ==========================================================================
// 静态解析函数
// ==========================================================================

Drive Hardware::parseDrive(const YAML::Node& node) {
    Drive d;
    d.id = node["id"].as<int>();
    d.joint_name = yamlGetStr(node, "joint_name", "joint_" + std::to_string(d.id));

    // 解析关节力矩传感器数据来源
    std::string src = yamlGetStr(node, "torque_source", "load_torque");
    if (src == "secondary_position") {
        d.torque_source = TorqueSource::SecondaryPosition;
    } else {
        d.torque_source = TorqueSource::LoadTorque;
    }

    // 解析 limit 子节点
    if (node["limit"]) {
        const auto& lim = node["limit"];
        d.limit.lower  = yamlGetDouble(lim, "lower", -2.71);
        d.limit.upper  = yamlGetDouble(lim, "upper", 2.71);
        d.limit.vel    = yamlGetDouble(lim, "vel", 3.0);
        d.limit.acc    = yamlGetDouble(lim, "acc", 10.0);
        d.limit.jerk   = yamlGetDouble(lim, "jerk", 150.0);
        d.limit.effort = yamlGetDouble(lim, "effort", 30.0);
    }

    // 解析 transform 子节点
    if (node["transform"]) {
        const auto& trans = node["transform"];
        d.transform.ratio          = yamlGetDouble(trans, "ratio", 1.0);
        d.transform.offset_pos_cnt = yamlGetInt(trans, "offset_pos_cnt", 0);
        d.transform.cnt_per_unit   = yamlGetDouble(trans, "cnt_per_unit", 1.0);
        d.transform.torque_per_unit = yamlGetDouble(trans, "torque_per_unit", 1.0);
        d.transform.user_unit_name = yamlGetStr(trans, "user_unit_name", "rad");
    }

    // 解析 inputs 子节点（PDO 变量名映射）
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        d.inputs.status_word              = yamlGetStr(in, "status_word");
        d.inputs.position_actual_value    = yamlGetStr(in, "position_actual_value");
        d.inputs.velocity_actual_value    = yamlGetStr(in, "velocity_actual_value");
        d.inputs.torque_actual_value      = yamlGetStr(in, "torque_actual_value");
        d.inputs.load_torque_value        = yamlGetStr(in, "load_torque_value");
        d.inputs.secondary_position_value = yamlGetStr(in, "secondary_position_value");
        d.inputs.secondary_velocity_value = yamlGetStr(in, "secondary_velocity_value");
        d.inputs.digital_inputs           = yamlGetStr(in, "digital_inputs");
        d.inputs.digital_outputs          = yamlGetStr(in, "digital_outputs");
    }

    // 解析 outputs 子节点（PDO 变量名映射）
    if (node["outputs"]) {
        const auto& out = node["outputs"];
        d.outputs.control_word      = yamlGetStr(out, "control_word");
        d.outputs.mode_of_operation = yamlGetStr(out, "mode_of_operation");
        d.outputs.target_position   = yamlGetStr(out, "target_position");
        d.outputs.target_velocity   = yamlGetStr(out, "target_velocity");
        d.outputs.target_torque     = yamlGetStr(out, "target_torque");
        d.outputs.digital_outputs   = yamlGetStr(out, "digital_outputs");
    }

    return d;
}

FTSensor Hardware::parseFTSensor(const YAML::Node& node) {
    FTSensor ft;
    ft.id = node["id"].as<int>();

    // 解析 6 通道 PDO 变量名映射
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        ft.inputs.fx = yamlGetStr(in, "fx");
        ft.inputs.fy = yamlGetStr(in, "fy");
        ft.inputs.fz = yamlGetStr(in, "fz");
        ft.inputs.tx = yamlGetStr(in, "tx");
        ft.inputs.ty = yamlGetStr(in, "ty");
        ft.inputs.tz = yamlGetStr(in, "tz");
    }

    // 解析标定偏置值
    if (node["offset"]) {
        const auto& off = node["offset"];
        ft.offset = Wrench(
            KDL::Vector(yamlGetDouble(off, "fx", 0.0),
                        yamlGetDouble(off, "fy", 0.0),
                        yamlGetDouble(off, "fz", 0.0)),
            KDL::Vector(yamlGetDouble(off, "tx", 0.0),
                        yamlGetDouble(off, "ty", 0.0),
                        yamlGetDouble(off, "tz", 0.0))
        );
    }

    return ft;
}

IO Hardware::parseIO(const YAML::Node& node) {
    IO io;
    io.id = node["id"].as<int>();

    // IO 通道数
    io.digital_in_channels  = yamlGetInt(node, "digital_in_channels", 0);
    io.digital_out_channels = yamlGetInt(node, "digital_out_channels", 0);
    io.analog_in_channels   = yamlGetInt(node, "analog_in_channels", 0);
    io.analog_out_channels  = yamlGetInt(node, "analog_out_channels", 0);

    // 输入 PDO 映射
    if (node["inputs"]) {
        const auto& in = node["inputs"];
        io.inputs.digital_inputs  = yamlGetStr(in, "digital_inputs");
        io.inputs.digital_outputs = yamlGetStr(in, "digital_outputs");
        io.inputs.analog_inputs   = yamlGetStr(in, "analog_inputs");
    }

    // 输出 PDO 映射
    if (node["outputs"]) {
        const auto& out = node["outputs"];
        io.outputs.digital_outputs = yamlGetStr(out, "digital_outputs");
        io.outputs.analog_outputs  = yamlGetStr(out, "analog_outputs");
    }

    return io;
}

// ==========================================================================
// 构造 / 析构
// ==========================================================================

Hardware::Hardware(const std::string& yaml_file_path, int ecat_id) {
    auto log_ptr = Logger::getInstance("Hardware");
    log_ptr->info("Loading hardware config from: {}", yaml_file_path);

    // 1. 加载 YAML 配置
    config_ = loadFromYAML(yaml_file_path);

    // 2. 构建 ID → 索引 O(1) 查找表（vector 预分配 + -1 哨兵）
    buildIDLookupTables();

    // 3. 连接 EtherCAT 共享内存
    ec_ptr_ = EcatConfig::getInstance(ecat_id);
    if (ec_ptr_ == nullptr) {
        log_ptr->error("Failed to get EcatConfig instance (id={})", ecat_id);
        throw std::runtime_error("EcatConfig initialization failed");
    }

    // 4. 等待 EtherCAT 总线进入 OP 状态
    ec_ptr_->wait();

    // 5. 初始化阶段：一次性解析所有 PDO 变量名 → 缓存指针
    //    运行阶段直接通过指针读写，不再有字符串查找开销
    buildPDOCache();

    log_ptr->info("Hardware initialized: {} drives, {} ft_sensors, {} io modules",
                  config_.drives.size(),
                  config_.ft_sensors.size(),
                  config_.ios.size());
}

Hardware::~Hardware() {
    // EcatConfig 是单例，不由 Hardware 管理生命周期
}

bool Hardware::Reset() {

  return true;
}

// ==========================================================================
// YAML 加载
// ==========================================================================

HardwareConfig Hardware::loadFromYAML(const std::string& yaml_file_path) {
    auto log_ptr = Logger::getInstance("Hardware");
    HardwareConfig config;

    try {
        YAML::Node root = YAML::LoadFile(yaml_file_path);

        // 顶层 key 为 "hardware"，是列表
        YAML::Node hw_list = root["hardware"];
        if (!hw_list || !hw_list.IsSequence()) {
            log_ptr->error("YAML file missing 'hardware' sequence: {}", yaml_file_path);
            throw std::runtime_error("Invalid hardware YAML: missing 'hardware' sequence");
        }

        for (const auto& node : hw_list) {
            if (!node["type"]) {
                log_ptr->warn("Skipping hardware entry without 'type' field");
                continue;
            }

            std::string type_str = node["type"].as<std::string>();

            if (type_str == "driver") {
                Drive d = parseDrive(node);
                log_ptr->info("  Parsed drive: id={}, joint={}", d.id, d.joint_name);
                config.drives.push_back(std::move(d));
            } else if (type_str == "ft_sensor") {
                FTSensor ft = parseFTSensor(node);
                log_ptr->info("  Parsed ft_sensor: id={}", ft.id);
                config.ft_sensors.push_back(std::move(ft));
            } else if (type_str == "io") {
                IO io = parseIO(node);
                log_ptr->info("  Parsed io: id={}, DI={}, DO={}, AI={}, AO={}",
                              io.id,
                              io.digital_in_channels,
                              io.digital_out_channels,
                              io.analog_in_channels,
                              io.analog_out_channels);
                config.ios.push_back(std::move(io));
            } else {
                log_ptr->warn("Unknown hardware type '{}' for id={}, skipping",
                              type_str,
                              yamlGetInt(node, "id", -1));
            }
        }

    } catch (const YAML::Exception& e) {
        log_ptr->error("YAML parse error in {}: {}", yaml_file_path, e.what());
        throw;
    }

    return config;
}

HardwareConfig Hardware::LoadConfigFromYAML(const std::string& yaml_file_path) {
    return loadFromYAML(yaml_file_path);
}

// ==========================================================================
// 坐标变换辅助函数
// ==========================================================================

double Hardware::cntToUnit(const Drive& drive, int32_t raw_cnt) const {
    // 物理值 = (原始值 - 偏移) / cnt_per_unit / ratio
    double raw = static_cast<double>(raw_cnt - drive.transform.offset_pos_cnt);
    return raw / drive.transform.cnt_per_unit / drive.transform.ratio;
}

int32_t Hardware::unitToCnt(const Drive& drive, double unit_val) const {
    // 原始值 = 物理值 * cnt_per_unit * ratio + 偏移
    double raw = unit_val * drive.transform.cnt_per_unit * drive.transform.ratio;
    return static_cast<int32_t>(raw) + drive.transform.offset_pos_cnt;
}

double Hardware::torqueToUnit(const Drive& drive, int16_t raw_torque) const {
    // 力矩物理值 = 原始值 / torque_per_unit
    return static_cast<double>(raw_torque) / drive.transform.torque_per_unit;
}

// ==========================================================================
// DriveInterface — 批量操作
// ==========================================================================

JntArray Hardware::GetPosition() {
    JntArray q(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        q(i) = GetJointPosition(config_.drives[i].id);
    }
    return q;
}

JntArray Hardware::GetVelocity() {
    JntArray q_dot(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        q_dot(i) = GetJointVelocity(config_.drives[i].id);
    }
    return q_dot;
}

JntArray Hardware::GetTorque() {
    JntArray tau(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        tau(i) = GetJointTorque(config_.drives[i].id);
    }
    return tau;
}

JntArray Hardware::GetLoadTorque() {
    JntArray lt(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        lt(i) = GetJointLoadTorque(config_.drives[i].id);
    }
    return lt;
}

void Hardware::SetPosition(const JntArray& q) {
    for (size_t i = 0; i < config_.drives.size() && i < q.rows(); ++i) {
        SetJointPosition(config_.drives[i].id, q(i));
    }
}

void Hardware::SetVelocity(const JntArray& q_dot) {
    for (size_t i = 0; i < config_.drives.size() && i < q_dot.rows(); ++i) {
        SetJointVelocity(config_.drives[i].id, q_dot(i));
    }
}

void Hardware::SetTorque(const JntArray& tau) {
    for (size_t i = 0; i < config_.drives.size() && i < tau.rows(); ++i) {
        SetJointTorque(config_.drives[i].id, tau(i));
    }
}

void Hardware::SetMode(int8_t mode) {
    for (auto& drive : config_.drives) {
        SetJointMode(drive.id, mode);
    }
}

void Hardware::SetEnabled() {
    for (auto& drive : config_.drives) {
        SetJointEnabled(drive.id);
    }
}

void Hardware::SetDisabled() {
    for (auto& drive : config_.drives) {
        SetJointDisabled(drive.id);
    }
}

// ==========================================================================
// DriveInterface — 单关节操作（使用缓存 PDO 指针，无字符串查找）
// ==========================================================================

double Hardware::GetJointPosition(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.position_actual_value == nullptr) return 0.0;
    return cntToUnit(config_.drives[static_cast<size_t>(idx)],
                     *cache.position_actual_value);
}

double Hardware::GetJointVelocity(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.velocity_actual_value == nullptr) return 0.0;
    return cntToUnit(config_.drives[static_cast<size_t>(idx)],
                     *cache.velocity_actual_value);
}

double Hardware::GetJointTorque(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.torque_actual_value == nullptr) return 0.0;
    return torqueToUnit(config_.drives[static_cast<size_t>(idx)],
                        *cache.torque_actual_value);
}

double Hardware::GetJointLoadTorque(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return 0.0;
    const auto& drive = config_.drives[static_cast<size_t>(idx)];
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];

    // 根据 torque_source 选择读取通道，对调用者透明
    if (drive.torque_source == TorqueSource::SecondaryPosition) {
        if (cache.secondary_position_value == nullptr) return 0.0;
        return cntToUnit(drive, *cache.secondary_position_value);
    } else {
        if (cache.load_torque_value == nullptr) return 0.0;
        return torqueToUnit(drive, *cache.load_torque_value);
    }
}

void Hardware::SetJointPosition(int32_t id, double pos) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.target_position == nullptr) return;
    *cache.target_position = unitToCnt(
        config_.drives[static_cast<size_t>(idx)], pos);
}

void Hardware::SetJointVelocity(int32_t id, double vel) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.target_velocity == nullptr) return;
    // 速度值也使用 cnt_per_unit 变换（单位: unit/s → cnt/s）
    double raw_val = vel * config_.drives[static_cast<size_t>(idx)].transform.cnt_per_unit
                     * config_.drives[static_cast<size_t>(idx)].transform.ratio;
    *cache.target_velocity = static_cast<int32_t>(raw_val);
}

void Hardware::SetJointTorque(int32_t id, double tau) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.target_torque == nullptr) return;
    *cache.target_torque = static_cast<int16_t>(
        tau * config_.drives[static_cast<size_t>(idx)].transform.torque_per_unit);
}

void Hardware::SetJointMode(int32_t id, int8_t mode) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
    if (cache.mode_of_operation == nullptr) return;
    *cache.mode_of_operation = mode;
}

void Hardware::SetJointEnabled(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    auto* cw = drive_pdo_cache_[static_cast<size_t>(idx)].control_word;
    if (cw == nullptr) return;
    // 写入 CiA 402 Controlword：Shutdown (0x06) → Switch On (0x07) → Enable Operation (0x0F)
    *cw = 0x0006;
    *cw = 0x0007;
    *cw = 0x000F;
}

void Hardware::SetJointDisabled(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return;
    auto* cw = drive_pdo_cache_[static_cast<size_t>(idx)].control_word;
    if (cw == nullptr) return;
    // 写入 CiA 402 Controlword：Disable Voltage (0x0000)
    *cw = 0x0000;
}

std::string Hardware::getJointName(int32_t id) {
    auto idx = getDriveIdx(id);
    if (idx < 0) return "";
    return config_.drives[static_cast<size_t>(idx)].joint_name;
}

// ==========================================================================
// FTSensorInterface（使用缓存 PDO 指针，无字符串查找）
// ==========================================================================

Wrench Hardware::GetWrench() {
    if (ft_sensor_pdo_cache_.empty()) {
        return Wrench::Zero();
    }

    const auto& cache = ft_sensor_pdo_cache_[0];
    const auto& ft = config_.ft_sensors[0];

    // 安全读取 PDO 通道：指针为空 → 返回 0.0
    auto safeRead = [](const int16_t* ptr) -> double {
        return ptr ? static_cast<double>(*ptr) : 0.0;
    };

    return Wrench(
        KDL::Vector(safeRead(cache.fx) + ft.offset.force.x(),
                    safeRead(cache.fy) + ft.offset.force.y(),
                    safeRead(cache.fz) + ft.offset.force.z()),
        KDL::Vector(safeRead(cache.tx) + ft.offset.torque.x(),
                    safeRead(cache.ty) + ft.offset.torque.y(),
                    safeRead(cache.tz) + ft.offset.torque.z())
    );
}

// ==========================================================================
// IOInteface（使用缓存 PDO 指针，无字符串查找）
// ==========================================================================

bool Hardware::GetDigitalInput(int32_t id, int32_t channel) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.digital_in_channels) return false;
            const auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.digital_inputs == nullptr) return false;
            return (*cache.digital_inputs >> channel) & 0x01;
        }
    }

    // 2. 在驱动器中查找（驱动器自带 DI）
    {
        auto idx = getDriveIdx(id);
        if (idx >= 0) {
            const auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.digital_inputs == nullptr) return false;
            return (*cache.digital_inputs >> channel) & 0x01;
        }
    }

    return false;
}

void Hardware::SetDigitalOutput(int32_t id, int32_t channel, bool value) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.digital_out_channels) return;
            auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.digital_outputs_input == nullptr
                || cache.digital_outputs == nullptr) return;
            int32_t current = *cache.digital_outputs_input;
            if (value) {
                current |= (1 << channel);
            } else {
                current &= ~(1 << channel);
            }
            *cache.digital_outputs = current;
            return;
        }
    }

    // 2. 在驱动器中查找（驱动器自带 DO）
    {
        auto idx = getDriveIdx(id);
        if (idx >= 0) {
            auto& cache = drive_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.digital_outputs_input == nullptr
                || cache.digital_outputs == nullptr) return;
            int32_t current = *cache.digital_outputs_input;
            if (value) {
                current |= (1 << channel);
            } else {
                current &= ~(1 << channel);
            }
            *cache.digital_outputs = current;
            return;
        }
    }
}

double Hardware::GetAnalogInput(int32_t id, int32_t channel) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.analog_in_channels) return 0.0;
            const auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.analog_inputs == nullptr) return 0.0;
            return static_cast<double>(*cache.analog_inputs);
        }
    }

    return 0.0;
}

void Hardware::SetAnalogOutput(int32_t id, int32_t channel, double value) {
    // 1. 先在 IO 模块中查找
    {
        auto idx = getIOIdx(id);
        if (idx >= 0) {
            const auto& io = config_.ios[static_cast<size_t>(idx)];
            if (channel < 0 || channel >= io.analog_out_channels) return;
            auto& cache = io_pdo_cache_[static_cast<size_t>(idx)];
            if (cache.analog_outputs == nullptr) return;
            *cache.analog_outputs = static_cast<int16_t>(value);
            return;
        }
    }
}

// ==========================================================================
// 查找表构建（初始化阶段）
// ==========================================================================

void Hardware::buildIDLookupTables() {
    // Lambda: 根据 items 的 id 构建 max_id+1 大小的 vector，哨兵值 -1
    auto build = [](const auto& items, std::vector<int32_t>& table) {
        table.clear();
        if (items.empty()) return;
        int32_t max_id = -1;
        for (const auto& item : items) {
            if (item.id > max_id) max_id = item.id;
        }
        if (max_id < 0) return;
        table.assign(static_cast<size_t>(max_id) + 1, -1);
        for (size_t i = 0; i < items.size(); ++i) {
            table[static_cast<size_t>(items[i].id)] = static_cast<int32_t>(i);
        }
    };

    build(config_.drives, drive_id_to_index_);
    build(config_.ft_sensors, ft_sensor_id_to_index_);
    build(config_.ios, io_id_to_index_);

    // 名称查找仍用 unordered_map（string key 无法用 vector）
    drive_name_to_index_.clear();
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        drive_name_to_index_[config_.drives[i].joint_name] = i;
    }
}

void Hardware::buildPDOCache() {
    auto log_ptr = Logger::getInstance("Hardware");

    // ========== 1. 驱动器 PDO 指针缓存 ==========
    drive_pdo_cache_.resize(config_.drives.size());
    for (size_t i = 0; i < config_.drives.size(); ++i) {
        const auto& drive = config_.drives[i];
        auto& cache = drive_pdo_cache_[i];
        const int sid = drive.id;

        // 输入 PDO（从站 → 主站）
        if (!drive.inputs.status_word.empty())
            cache.status_word = ec_ptr_->findSlaveInputVarPtrByName<uint16_t>(
                sid, drive.inputs.status_word);
        if (!drive.inputs.position_actual_value.empty())
            cache.position_actual_value = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.position_actual_value);
        if (!drive.inputs.velocity_actual_value.empty())
            cache.velocity_actual_value = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.velocity_actual_value);
        if (!drive.inputs.torque_actual_value.empty())
            cache.torque_actual_value = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(
                sid, drive.inputs.torque_actual_value);
        if (!drive.inputs.load_torque_value.empty())
            cache.load_torque_value = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(
                sid, drive.inputs.load_torque_value);
        if (!drive.inputs.secondary_position_value.empty())
            cache.secondary_position_value = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.secondary_position_value);
        if (!drive.inputs.secondary_velocity_value.empty())
            cache.secondary_velocity_value = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.secondary_velocity_value);
        if (!drive.inputs.digital_inputs.empty())
            cache.digital_inputs = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.digital_inputs);
        if (!drive.inputs.digital_outputs.empty())
            cache.digital_outputs_input = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, drive.inputs.digital_outputs);

        // 输出 PDO（主站 → 从站）
        if (!drive.outputs.control_word.empty())
            cache.control_word = ec_ptr_->findSlaveOutputVarPtrByName<uint16_t>(
                sid, drive.outputs.control_word);
        if (!drive.outputs.mode_of_operation.empty())
            cache.mode_of_operation = ec_ptr_->findSlaveOutputVarPtrByName<int8_t>(
                sid, drive.outputs.mode_of_operation);
        if (!drive.outputs.target_position.empty())
            cache.target_position = ec_ptr_->findSlaveOutputVarPtrByName<int32_t>(
                sid, drive.outputs.target_position);
        if (!drive.outputs.target_velocity.empty())
            cache.target_velocity = ec_ptr_->findSlaveOutputVarPtrByName<int32_t>(
                sid, drive.outputs.target_velocity);
        if (!drive.outputs.target_torque.empty())
            cache.target_torque = ec_ptr_->findSlaveOutputVarPtrByName<int16_t>(
                sid, drive.outputs.target_torque);
        if (!drive.outputs.digital_outputs.empty())
            cache.digital_outputs = ec_ptr_->findSlaveOutputVarPtrByName<int32_t>(
                sid, drive.outputs.digital_outputs);

        // 关键 PDO 变量未找到时记录警告
        if (cache.position_actual_value == nullptr) {
            log_ptr->warn("Drive {}: position_actual_value PDO ptr is null (var='{}')",
                          sid, drive.inputs.position_actual_value);
        }
        if (cache.control_word == nullptr) {
            log_ptr->warn("Drive {}: control_word PDO ptr is null (var='{}')",
                          sid, drive.outputs.control_word);
        }
    }

    // ========== 2. 力传感器 PDO 指针缓存 ==========
    ft_sensor_pdo_cache_.resize(config_.ft_sensors.size());
    for (size_t i = 0; i < config_.ft_sensors.size(); ++i) {
        const auto& ft = config_.ft_sensors[i];
        auto& cache = ft_sensor_pdo_cache_[i];
        const int sid = ft.id;

        if (!ft.inputs.fx.empty())
            cache.fx = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.fx);
        if (!ft.inputs.fy.empty())
            cache.fy = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.fy);
        if (!ft.inputs.fz.empty())
            cache.fz = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.fz);
        if (!ft.inputs.tx.empty())
            cache.tx = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.tx);
        if (!ft.inputs.ty.empty())
            cache.ty = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.ty);
        if (!ft.inputs.tz.empty())
            cache.tz = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(sid, ft.inputs.tz);
    }

    // ========== 3. IO 模块 PDO 指针缓存 ==========
    io_pdo_cache_.resize(config_.ios.size());
    for (size_t i = 0; i < config_.ios.size(); ++i) {
        const auto& io = config_.ios[i];
        auto& cache = io_pdo_cache_[i];
        const int sid = io.id;

        if (!io.inputs.digital_inputs.empty())
            cache.digital_inputs = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, io.inputs.digital_inputs);
        if (!io.inputs.digital_outputs.empty())
            cache.digital_outputs_input = ec_ptr_->findSlaveInputVarPtrByName<int32_t>(
                sid, io.inputs.digital_outputs);
        if (!io.inputs.analog_inputs.empty())
            cache.analog_inputs = ec_ptr_->findSlaveInputVarPtrByName<int16_t>(
                sid, io.inputs.analog_inputs);
        if (!io.outputs.digital_outputs.empty())
            cache.digital_outputs = ec_ptr_->findSlaveOutputVarPtrByName<int32_t>(
                sid, io.outputs.digital_outputs);
        if (!io.outputs.analog_outputs.empty())
            cache.analog_outputs = ec_ptr_->findSlaveOutputVarPtrByName<int16_t>(
                sid, io.outputs.analog_outputs);
    }
}

// ==========================================================================
// 配置查询
// ==========================================================================

const Drive* Hardware::findDriveById(int32_t id) const {
    auto idx = getDriveIdx(id);
    return (idx >= 0) ? &config_.drives[static_cast<size_t>(idx)] : nullptr;
}

const Drive* Hardware::findDriveByName(const std::string& name) const {
    auto it = drive_name_to_index_.find(name);
    if (it == drive_name_to_index_.end()) {
        return nullptr;
    }
    return &config_.drives[it->second];
}

const FTSensor* Hardware::findFTSensorById(int32_t id) const {
    auto idx = getFTSensorIdx(id);
    return (idx >= 0) ? &config_.ft_sensors[static_cast<size_t>(idx)] : nullptr;
}

const IO* Hardware::findIOById(int32_t id) const {
    auto idx = getIOIdx(id);
    return (idx >= 0) ? &config_.ios[static_cast<size_t>(idx)] : nullptr;
}

HardwareType Hardware::getHardwareType(int32_t slave_id) const {
    if (getDriveIdx(slave_id) >= 0) {
        return HardwareType::Driver;
    }
    if (getFTSensorIdx(slave_id) >= 0) {
        return HardwareType::FTSensor;
    }
    if (getIOIdx(slave_id) >= 0) {
        return HardwareType::IO;
    }
    // 默认返回 Driver 类型
    return HardwareType::Driver;
}

}  // namespace rocos
