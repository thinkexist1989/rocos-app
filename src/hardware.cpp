//
// Created by think on 6/30/26.
//

#include "hardware.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
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
    }

    return d;
}

FTSensor Hardware::parseFTSensor(const YAML::Node& node) {
    FTSensor ft;
    ft.id = node["id"].as<int>();

    // 解析六维力/力矩偏置/补偿值
    if (node["wrench"]) {
        const auto& wr = node["wrench"];
        ft.wrench = Wrench(
            KDL::Vector(yamlGetDouble(wr, "fx", 0.0),
                        yamlGetDouble(wr, "fy", 0.0),
                        yamlGetDouble(wr, "fz", 0.0)),
            KDL::Vector(yamlGetDouble(wr, "tx", 0.0),
                        yamlGetDouble(wr, "ty", 0.0),
                        yamlGetDouble(wr, "tz", 0.0))
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
    loadFromYAML(yaml_file_path);

    // 2. 连接 EtherCAT 共享内存
    ec_ptr_ = EcatConfig::getInstance(ecat_id);
    if (ec_ptr_ == nullptr) {
        log_ptr->error("Failed to get EcatConfig instance (id={})", ecat_id);
        throw std::runtime_error("EcatConfig initialization failed");
    }

    // 3. 等待 EtherCAT 总线进入 OP 状态
    ec_ptr_->wait();

    log_ptr->info("Hardware initialized: {} drives, {} ft_sensors, {} io modules",
                  config_.drives.size(),
                  config_.ft_sensors.size(),
                  config_.ios.size());
}

Hardware::~Hardware() {
    // EcatConfig 是单例，不由 Hardware 管理生命周期
}

// ==========================================================================
// YAML 加载
// ==========================================================================

void Hardware::loadFromYAML(const std::string& yaml_file_path) {
    // auto log_ptr = Logger::getInstance("Hardware");

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
                config_.drives.push_back(std::move(d));
            } else if (type_str == "ft_sensor") {
                FTSensor ft = parseFTSensor(node);
                log_ptr->info("  Parsed ft_sensor: id={}", ft.id);
                config_.ft_sensors.push_back(std::move(ft));
            } else if (type_str == "io") {
                IO io = parseIO(node);
                log_ptr->info("  Parsed io: id={}, DI={}, DO={}, AI={}, AO={}",
                              io.id,
                              io.digital_in_channels,
                              io.digital_out_channels,
                              io.analog_in_channels,
                              io.analog_out_channels);
                config_.ios.push_back(std::move(io));
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
// DriveInterface — 单关节操作
// ==========================================================================

double Hardware::GetJointPosition(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return 0.0;
    }
    // 注意：输入 PDO（从站→主站）使用 getSlaveInputVarValueByName
    int32_t raw = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
        id, drive->inputs.position_actual_value);
    return cntToUnit(*drive, raw);
}

double Hardware::GetJointVelocity(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return 0.0;
    }
    int32_t raw = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
        id, drive->inputs.velocity_actual_value);
    return cntToUnit(*drive, raw);
}

double Hardware::GetJointTorque(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return 0.0;
    }
    int16_t raw = ec_ptr_->getSlaveInputVarValueByName<int16_t>(
        id, drive->inputs.torque_actual_value);
    return torqueToUnit(*drive, raw);
}

double Hardware::GetJointLoadTorque(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return 0.0;
    }
    // 根据 torque_source 选择读取通道，对调用者透明
    if (drive->torque_source == TorqueSource::SecondaryPosition) {
        int32_t raw = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
            id, drive->inputs.secondary_position_value);
        return cntToUnit(*drive, raw);   // 底层已滤波，按位置变换
    } else {
        int16_t raw = ec_ptr_->getSlaveInputVarValueByName<int16_t>(
            id, drive->inputs.load_torque_value);
        return torqueToUnit(*drive, raw);  // Analog Input 原始值
    }
}

void Hardware::SetJointPosition(int32_t id, double pos) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    int32_t raw = unitToCnt(*drive, pos);
    ec_ptr_->setSlaveOutputVarValueByName<int32_t>(
        id, drive->outputs.target_position, raw);
}

void Hardware::SetJointVelocity(int32_t id, double vel) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    // 速度值也使用 cnt_per_unit 变换（单位: unit/s → cnt/s）
    double raw_val = vel * drive->transform.cnt_per_unit * drive->transform.ratio;
    int32_t raw = static_cast<int32_t>(raw_val);
    ec_ptr_->setSlaveOutputVarValueByName<int32_t>(
        id, drive->outputs.target_velocity, raw);
}

void Hardware::SetJointTorque(int32_t id, double tau) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    int16_t raw = static_cast<int16_t>(tau * drive->transform.torque_per_unit);
    ec_ptr_->setSlaveOutputVarValueByName<int16_t>(
        id, drive->outputs.target_torque, raw);
}

void Hardware::SetJointMode(int32_t id, int8_t mode) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    ec_ptr_->setSlaveOutputVarValueByName<int8_t>(
        id, drive->outputs.mode_of_operation, mode);
}

void Hardware::SetJointEnabled(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    // 写入 CiA 402 Controlword：Shutdown (0x06) → Switch On (0x07) → Enable Operation (0x0F)
    ec_ptr_->setSlaveOutputVarValueByName<uint16_t>(id, drive->outputs.control_word, 0x0006);
    ec_ptr_->setSlaveOutputVarValueByName<uint16_t>(id, drive->outputs.control_word, 0x0007);
    ec_ptr_->setSlaveOutputVarValueByName<uint16_t>(id, drive->outputs.control_word, 0x000F);
}

void Hardware::SetJointDisabled(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return;
    }
    // 写入 CiA 402 Controlword：Disable Voltage (0x0000)
    ec_ptr_->setSlaveOutputVarValueByName<uint16_t>(id, drive->outputs.control_word, 0x0000);
}

std::string Hardware::getJointName(int32_t id) {
    const Drive* drive = findDriveById(id);
    if (drive == nullptr) {
        return "";
    }
    return drive->joint_name;
}

// ==========================================================================
// FTSensorInterface
// ==========================================================================

Wrench Hardware::GetWrench() {
    if (config_.ft_sensors.empty()) {
        return Wrench::Zero();
    }

    // 取第一个力传感器数据，加上配置中的偏置 Wrench
    const auto& ft = config_.ft_sensors[0];

    // TODO: 从 FTSensor 从站 PDO 读取完整的 6 维原始数据并解算
    // 当前返回配置中的偏置值（用于标定场景）
    return ft.wrench;
}

// ==========================================================================
// IOInteface
// ==========================================================================

bool Hardware::GetDigitalInput(int id, int channel) {
    // 1. 先在 IO 模块中查找
    const IO* io = findIOById(id);
    if (io != nullptr) {
        if (channel < 0 || channel >= io->digital_in_channels) {
            return false;
        }
        int32_t di = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
            id, io->inputs.digital_inputs);
        return (di >> channel) & 0x01;
    }

    // 2. 在驱动器中查找（驱动器自带 DI）
    const Drive* drive = findDriveById(id);
    if (drive != nullptr && !drive->inputs.digital_inputs.empty()) {
        int32_t di = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
            id, drive->inputs.digital_inputs);
        return (di >> channel) & 0x01;
    }

    return false;
}

void Hardware::SetDigitalOutput(int id, int channel, bool value) {
    // 1. 先在 IO 模块中查找
    const IO* io = findIOById(id);
    if (io != nullptr) {
        if (channel < 0 || channel >= io->digital_out_channels) {
            return;
        }
        // 从输入 PDO 回读当前 DO 实际状态，修改指定位后写入输出 PDO
        int32_t current = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
            id, io->inputs.digital_outputs);
        if (value) {
            current |= (1 << channel);
        } else {
            current &= ~(1 << channel);
        }
        ec_ptr_->setSlaveOutputVarValueByName<int32_t>(
            id, io->outputs.digital_outputs, current);
        return;
    }

    // 2. 在驱动器中查找（驱动器自带 DO）
    const Drive* drive = findDriveById(id);
    if (drive != nullptr && !drive->outputs.control_word.empty()) {
        // 驱动器通过 PDO input 区回读 DO 实际状态
        int32_t current = ec_ptr_->getSlaveInputVarValueByName<int32_t>(
            id, drive->inputs.digital_outputs);
        if (value) {
            current |= (1 << channel);
        } else {
            current &= ~(1 << channel);
        }
        // 驱动器 DO 可能通过 PDO output 区域控制，也可能通过 SDO
        // 这里仅做占位实现
    }
}

double Hardware::GetAnalogInput(int id, int channel) {
    // 1. 先在 IO 模块中查找
    const IO* io = findIOById(id);
    if (io != nullptr) {
        if (channel < 0 || channel >= io->analog_in_channels) {
            return 0.0;
        }
        // TODO: 根据具体 IO 模块的 AI 数组读取实现
        int16_t raw = ec_ptr_->getSlaveInputVarValueByName<int16_t>(
            id, io->inputs.analog_inputs);
        return static_cast<double>(raw);
    }

    return 0.0;
}

void Hardware::SetAnalogOutput(int id, int channel, double value) {
    // 1. 先在 IO 模块中查找
    const IO* io = findIOById(id);
    if (io != nullptr) {
        if (channel < 0 || channel >= io->analog_out_channels) {
            return;
        }
        // TODO: 根据具体 IO 模块的 AO 数组写入实现
        int16_t raw = static_cast<int16_t>(value);
        ec_ptr_->setSlaveOutputVarValueByName<int16_t>(
            id, io->outputs.analog_outputs, raw);
        return;
    }
}

// ==========================================================================
// 配置查询
// ==========================================================================

const Drive* Hardware::findDriveById(int id) const {
    auto it = std::find_if(config_.drives.begin(), config_.drives.end(),
                           [id](const Drive& d) { return d.id == id; });
    return (it != config_.drives.end()) ? &(*it) : nullptr;
}

const Drive* Hardware::findDriveByName(const std::string& name) const {
    auto it = std::find_if(config_.drives.begin(), config_.drives.end(),
                           [&name](const Drive& d) { return d.joint_name == name; });
    return (it != config_.drives.end()) ? &(*it) : nullptr;
}

const FTSensor* Hardware::findFTSensorById(int id) const {
    auto it = std::find_if(config_.ft_sensors.begin(), config_.ft_sensors.end(),
                           [id](const FTSensor& ft) { return ft.id == id; });
    return (it != config_.ft_sensors.end()) ? &(*it) : nullptr;
}

const IO* Hardware::findIOById(int id) const {
    auto it = std::find_if(config_.ios.begin(), config_.ios.end(),
                           [id](const IO& io) { return io.id == id; });
    return (it != config_.ios.end()) ? &(*it) : nullptr;
}

HardwareType Hardware::getHardwareType(int slave_id) const {
    if (findDriveById(slave_id) != nullptr) {
        return HardwareType::Driver;
    }
    if (findFTSensorById(slave_id) != nullptr) {
        return HardwareType::FTSensor;
    }
    if (findIOById(slave_id) != nullptr) {
        return HardwareType::IO;
    }
    // 默认返回 Driver 类型
    return HardwareType::Driver;
}

}  // namespace rocos
