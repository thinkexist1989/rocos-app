#ifndef DHPARAMS_LOADER_H
#define DHPARAMS_LOADER_H
#include <iomanip>   // ← 把这行放到文件顶部
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/jntarray.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <memory>
#include <iostream>

#include <rocos_app/logger.h>

struct DHParameters {
    std::string joint_name;
    double alpha;  // alpha(i-1)
    double a;      // a(i-1)  
    double d;      // d(i)
    double theta;  // theta(i)
    double offset; // 偏移量
    std::string type;  // "revolute" or "prismatic"
    // std::vector<double> limits;  // [min, max]
    
    DHParameters() : alpha(0.0), a(0.0), d(0.0), theta(0.0), offset(0.0), type("revolute") {}
    // DHParameters() : alpha(0.0), a(0.0), d(0.0), theta(0.0), type("revolute") {}
};

class DHParamsLoader {
public:
    DHParamsLoader();
    ~DHParamsLoader();
    
    /**
     * @brief 从YAML文件加载DH参数
     * @param yaml_file YAML文件路径
     * @return 是否加载成功
     */
    bool loadFromYAML(const std::string& yaml_file);
    
    /**
     * @brief 获取构建的运动学链
     * @return KDL运动学链
     */
    KDL::Chain getChain() const { return chain_; }
    
    /**
     * @brief 获取关节下限
     * @return 关节下限数组
     */
    // KDL::JntArray getJointMin() const { return q_min_; }
    
    /**
     * @brief 获取关节上限
     * @return 关节上限数组
     */
    // KDL::JntArray getJointMax() const { return q_max_; }
    
    /**
     * @brief 获取基座标系名称
     * @return 基座标系名称
     */
    std::string getBaseLink() const { return base_link_; }
    
    /**
     * @brief 获取末端坐标系名称
     * @return 末端坐标系名称
     */
    std::string getTipLink() const { return tip_link_; }
    
    /**
     * @brief 获取机器人名称
     * @return 机器人名称
     */
    std::string getRobotName() const { return robot_name_; }
    
    /**
     * @brief 获取DH参数列表
     * @return DH参数向量
     */
    std::vector<DHParameters> getDHParameters() const { return dh_params_; }
    
    /**
     * @brief 打印加载的DH参数信息
     */
    void printDHParameters() const;
    /**
     * @brief 打印变换矩阵
     * @param frame 变换矩阵
     */
    void printFrame(const KDL::Frame& frame) const;
private:
    /**
     * @brief 从加载的DH参数构建运动学链
     * @return 是否构建成功
     */
    bool buildChainFromDH();
    
    /**
     * @brief 根据DH参数创建KDL关节
     * @param dh DH参数
     * @return KDL关节
     */
    KDL::Joint createJoint(const DHParameters& dh);
    
    /**
     * @brief 根据DH参数创建变换矩阵
     * @param dh DH参数
     * @return 变换矩阵
     */
    KDL::Frame createFrame(const DHParameters& dh);
    
    /**
     * @brief 初始化关节限位
     */
    void initializeJointLimits();
    

private:
    std::string robot_name_;
    std::string base_link_;
    std::string tip_link_;
    std::vector<DHParameters> dh_params_;
    
    KDL::Chain chain_;
    // KDL::JntArray q_min_;
    // KDL::JntArray q_max_;
    bool is_standard_dh_ = false;
    bool is_mdh_ = false;

    Logger::logger_ptr log_ptr_ = nullptr;
};

#endif // DHPARAMS_LOADER_H