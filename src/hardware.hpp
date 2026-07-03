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

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface.hpp"
#include "ecat_config.hpp"


// 前向声明，避免在头文件中引入 yaml-cpp
namespace YAML {
class Node;
}

namespace rocos {

// ==========================================================================
// 枚举定义
// ==========================================================================

/// @brief 扭矩传感器数据来源 —— 对上层使用者透明
enum class TorqueSource {
    LoadTorque,         // 通过 load_torque_value（int16 Analog Input 原始值）
    SecondaryPosition   // 通过 secondary_position_value（底层已做滤波）
};

/// @brief 硬件模块类型
enum class HardwareType {
    Driver,     // 伺服驱动器
    FTSensor,   // 力/力矩传感器
    IO          // 独立 IO 模块
};

// ==========================================================================
// 驱动器配置结构体
// ==========================================================================
struct Drive {
    int id{-1};                        // EtherCAT 从站 ID
    std::string joint_name;            // 对应的关节名称

    /// @brief 关节力矩传感器数据来源 —— 对使用者透明
    /// load_torque:    从 load_torque_value 读取 int16 原始 Analog Input
    /// secondary_pos:  从 secondary_position_value 读取底层滤波后的数据
    TorqueSource torque_source = TorqueSource::LoadTorque;

    /// @brief 运动限位参数
    struct Limit {
        double lower = -2.71;          // 位置下限 [rad] 或 [m]
        double upper = 2.71;           // 位置上限 [rad] 或 [m]
        double vel = 3.0;              // 最大速度
        double acc = 10.0;             // 最大加速度
        double jerk = 150.0;           // 最大加加速度
        double effort = 30.0;          // 最大力矩/力
    } limit;

    /// @brief 编码器/力矩变换参数（cnt ↔ 物理单位）
    struct Transform {
        double ratio = 1.0;            // 传动比（关节侧/电机侧）
        int offset_pos_cnt = 0;        // 编码器零点偏移 [cnt]
        double cnt_per_unit = 1.0;     // 每物理单位对应编码器计数
        double torque_per_unit = 1.0;  // 每物理单位对应力矩值
        std::string user_unit_name = "rad";  // 用户单位名称
    } transform;

    /// @brief 输入 PDO 变量名（从站 → 主站），需与 ethercat 配置中的名称一致
    struct InputVar {
        std::string status_word;
        std::string position_actual_value;
        std::string velocity_actual_value;
        std::string torque_actual_value;
        std::string load_torque_value;           // 负载力矩（可用于力矩传感器）
        std::string secondary_position_value;    // 辅助位置（可用于底层滤波后的力矩）
        std::string secondary_velocity_value;
        std::string digital_inputs;              // 数字输入（DI）
        std::string digital_outputs;             // 数字输出回读（DO 状态反馈）
    } inputs;

    /// @brief 输出 PDO 变量名（主站 → 从站），需与 ethercat 配置中的名称一致
    struct OutputVar {
        std::string control_word;
        std::string mode_of_operation;
        std::string target_position;
        std::string target_velocity;
        std::string target_torque;
        std::string digital_outputs;             // 数字输出（DO）
    } outputs;

};

// ==========================================================================
// 力/力矩传感器配置结构体
// ==========================================================================
struct FTSensor {
    int id{-1};                        // EtherCAT 从站 ID

    /// @brief 输入 PDO 变量名（从站 → 主站），6 通道映射
    struct InputVar {
        std::string fx;    // Fx 通道
        std::string fy;    // Fy 通道
        std::string fz;    // Fz 通道
        std::string tx;    // Tx 通道
        std::string ty;    // Ty 通道
        std::string tz;    // Tz 通道
    } inputs;

    /// @brief 六维力/力矩标定偏置值 (Fx,Fy,Fz,Tx,Ty,Tz)
    Wrench offset;
};

// ==========================================================================
// IO 模块配置结构体
// ==========================================================================
struct IO {
    int id{-1};                        // EtherCAT 从站 ID

    /// @brief IO 通道数量（用于参数校验与能力查询）
    int digital_in_channels = 0;       // DI 通道数
    int digital_out_channels = 0;      // DO 通道数
    int analog_in_channels = 0;        // AI 通道数
    int analog_out_channels = 0;       // AO 通道数

    /// @brief 输入 PDO 变量名（从站 → 主站）
    struct InputVar {
        std::string digital_inputs;    // DI 值（位掩码）
        std::string digital_outputs;   // DO 回读（位掩码）
        std::string analog_inputs;     // AI 值（通常为 int16 数组）
    } inputs;

    /// @brief 输出 PDO 变量名（主站 → 从站）
    struct OutputVar {
        std::string digital_outputs;   // DO 值（位掩码）
        std::string analog_outputs;    // AO 值（通常为 int16 数组）
    } outputs;
};

// ==========================================================================
// 硬件配置（YAML 顶层结构）
// ==========================================================================
struct HardwareConfig {
    std::vector<Drive> drives;         // 所有伺服驱动器
    std::vector<FTSensor> ft_sensors;  // 所有力/力矩传感器
    std::vector<IO> ios;               // 所有独立 IO 模块
};

// ==========================================================================
// 硬件抽象层
// ==========================================================================
class Hardware : public HardwareInterface {
public:
    /// @brief 从 YAML 配置文件构造硬件抽象层
    /// @param yaml_file_path 硬件描述 YAML 文件路径
    /// @param ecat_id         EtherCAT 主站 ID（默认 0）
    explicit Hardware(const std::string& yaml_file_path, int ecat_id = 0);
    ~Hardware();

    bool Reset() override;

    // ========== DriveInterface 接口实现 ==========
    JntArray GetPosition() override;
    JntArray GetVelocity() override;
    JntArray GetTorque() override;
    JntArray GetLoadTorque() override;
    void SetPosition(const JntArray& q) override;
    void SetVelocity(const JntArray& q_dot) override;
    void SetTorque(const JntArray& tau) override;
    void SetMode(int8_t mode) override;
    void SetEnabled() override;
    void SetDisabled() override;
    double GetJointPosition(int32_t id) override;
    double GetJointVelocity(int32_t id) override;
    double GetJointTorque(int32_t id) override;
    double GetJointLoadTorque(int32_t id) override;
    void SetJointPosition(int32_t id, double pos) override;
    void SetJointVelocity(int32_t id, double vel) override;
    void SetJointTorque(int32_t id, double tau) override;
    void SetJointMode(int32_t id, int8_t mode) override;
    void SetJointEnabled(int32_t id) override;
    void SetJointDisabled(int32_t id) override;
    std::string getJointName(int32_t id) override;

    // ========== FTSensorInterface 接口实现 ==========
    Wrench GetWrench() override;

    // ========== IOInteface 接口实现 ==========
    bool GetDigitalInput(int id, int channel) override;
    void SetDigitalOutput(int id, int channel, bool value) override;
    double GetAnalogInput(int id, int channel) override;
    void SetAnalogOutput(int id, int channel, double value) override;

    // ========== 配置访问 ==========
    const HardwareConfig& getConfig() const { return config_; }

    /// @brief 根据从站 ID 查找驱动
    const Drive* findDriveById(int id) const;
    /// @brief 根据关节名查找驱动
    const Drive* findDriveByName(const std::string& name) const;
    /// @brief 根据从站 ID 查找力矩传感器
    const FTSensor* findFTSensorById(int id) const;
    /// @brief 根据从站 ID 查找 IO 模块
    const IO* findIOById(int id) const;
    /// @brief 获取硬件类型
    HardwareType getHardwareType(int slave_id) const;

    /// @brief [测试用] 从 YAML 文件加载并返回配置，不初始化 EcatConfig
    static HardwareConfig LoadConfigFromYAML(const std::string& yaml_file_path);

private:
    /// @brief 从 YAML 文件加载硬件配置
    static HardwareConfig loadFromYAML(const std::string& yaml_file_path);

    /// @brief 从 YAML 节点解析单个 Drive
    static Drive parseDrive(const YAML::Node& node);
    /// @brief 从 YAML 节点解析单个 FTSensor
    static FTSensor parseFTSensor(const YAML::Node& node);
    /// @brief 从 YAML 节点解析单个 IO 模块
    static IO parseIO(const YAML::Node& node);

    /// @brief 将编码器原始值转换为物理单位
    double cntToUnit(const Drive& drive, int32_t raw_cnt) const;
    /// @brief 将物理单位转换为编码器原始值
    int32_t unitToCnt(const Drive& drive, double unit_val) const;

    /// @brief 将力矩原始值转换为物理单位
    double torqueToUnit(const Drive& drive, int16_t raw_torque) const;

    HardwareConfig config_;           // 硬件配置
    EcatConfig* ec_ptr_{nullptr};     // EtherCAT 配置单例指针（非拥有）
};

}  // namespace rocos
