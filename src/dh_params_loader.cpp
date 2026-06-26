#include "dh_params_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <kdl/frames.hpp>

namespace rocos {

DHParamsLoader::DHParamsLoader() {
    // 初始化默认值
    robot_name_ = "Unknown";
    base_link_ = "base_link";
    tip_link_ = "tool0";

    log_ptr_ = Logger::getInstance("DHParamsLoader");
}

DHParamsLoader::~DHParamsLoader() {
    // 析构函数
}

bool DHParamsLoader::loadFromYAML(const std::string& yaml_file) {
    try {
        YAML::Node config = YAML::LoadFile(yaml_file);
        
        // 读取基本信息
        robot_name_ = config["robot_name"].as<std::string>("Unknown");
        base_link_ = config["base_link"].as<std::string>("base_link");
        tip_link_ = config["tip_link"].as<std::string>("wrist_3_link");
        is_standard_dh_ = (config["dh_convention"].as<std::string>("standard") == "standard");
        is_mdh_ = (config["dh_convention"].as<std::string>("standard") == "modified");

        log_ptr_->info("Loading robot: {}", robot_name_);
        log_ptr_->info("Base link: {}", base_link_);
        log_ptr_->info("Tip link: {}", tip_link_);
        log_ptr_->info("DH Convention: {}", (is_standard_dh_ ? "Standard DH" : (is_mdh_ ? "Modified DH" : "Unknown")));
        
        // 清空已有的DH参数
        dh_params_.clear();
        
        // 读取DH参数
        YAML::Node dh_nodes = config["dh_parameters"];
        if (!dh_nodes) {
            log_ptr_->error("No 'dh_parameters' found in YAML file!");
            return false;
        }
        
        for (const auto& node : dh_nodes) {
            DHParameters dh;
            dh.joint_name = node["joint_name"].as<std::string>();
            dh.alpha = node["alpha"].as<double>();
            dh.a = node["a"].as<double>();
            dh.d = node["d"].as<double>();
            dh.theta = node["theta"].as<double>();
            dh.type = node["type"].as<std::string>("revolute");
            dh.offset = node["offset"].as<double>(0.0);

            
            
            dh_params_.push_back(dh);

            log_ptr_->info("Loaded joint: {} (type: {})", dh.joint_name, dh.type);

        }
        
        log_ptr_->info("Successfully loaded {} DH parameters", dh_params_.size());

        
        // 构建运动学链
        return buildChainFromDH();
        
    } catch (const YAML::Exception& e) {
        log_ptr_->error("Failed to load DH parameters yaml file : {} => {}", yaml_file, e.what());
        return false;
    } catch (const std::exception& e) {
        log_ptr_->error("Exception while loading DH parameters yaml file : {} => {}", yaml_file, e.what());
        return false;
    }
}

bool DHParamsLoader::buildChainFromDH() {
    chain_ = KDL::Chain();
    
    if (dh_params_.empty()) {
        log_ptr_->error("No DH parameters available to build chain!");
        return false;
    }
    
    log_ptr_->info("Building kinematic chain from {} DH parameters", dh_params_.size());

    
    // 为每个DH参数添加关节和连杆
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        auto& dh = dh_params_[i];
        
        // 创建关节
        if(i==0)
        {
            dh.type="none";
        }
        KDL::Joint joint = createJoint(dh);
        
        // 创建变换矩阵（改进DH变换）
        KDL::Frame frame = createFrame(dh);
        
        // 添加Segment到链中
        chain_.addSegment(KDL::Segment(dh.joint_name + "_link", joint, frame));

    }

    chain_.addSegment(KDL::Segment("end_effector", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::Identity()));

    log_ptr_->info("Successfully built kinematic chain with {} joints", chain_.getNrOfJoints());
    
    return true;
}

KDL::Joint DHParamsLoader::createJoint(const DHParameters& dh) {
   if (dh.type == "revolute") {
        // 使用预定义的旋转关节类型，绕Z轴旋转
        return KDL::Joint(dh.joint_name, KDL::Joint::RotZ);
    } else if (dh.type == "prismatic") {
        // 使用预定义的平移关节类型，沿Z轴平移
        return KDL::Joint(dh.joint_name, KDL::Joint::TransZ);
    } else {
        log_ptr_->warn("Unknown joint type '{}' for joint '{}', using fixed joint instead.", dh.type, dh.joint_name);
        return KDL::Joint(dh.joint_name,KDL::Joint::None);
    }
}

KDL::Frame DHParamsLoader::createFrame(const DHParameters& dh) {
    // 改进DH参数变换顺序: 
    // RotX(alpha(i-1)) * TransX(a(i-1)) * RotZ(theta(i)) * TransZ(d(i))
    
    KDL::Frame frame = KDL::Frame::Identity();
    if (is_mdh_) {
        // 改进DH参数变换顺序 (Modified DH): 
        // RotX(alpha(i-1)) * TransX(a(i-1)) * RotZ(theta(i)) * TransZ(d(i))

        double q = dh.theta + dh.offset;   // 统一先算好

        // 1. 绕X轴旋转alpha(i-1)
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));
        // 2. 沿X轴平移a(i-1)
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        // 3. 绕Z轴旋转theta(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(q ));
        // 4. 沿Z轴平移d(i)
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));

        return KDL::Frame::DH_Craig1989(dh.a, dh.alpha, dh.d, q );
    } 
    else if (is_standard_dh_) {
        // 标准DH参数变换顺序 (Standard DH):
        // RotZ(theta(i)) * TransZ(d(i)) * TransX(a(i)) * RotX(alpha(i))

        // 1. 绕Z轴旋转theta(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(dh.theta + dh.offset));
        // 2. 沿Z轴平移d(i)
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));
        // 3. 沿X轴平移a(i)
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        // 4. 绕X轴旋转alpha(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));

        return KDL::Frame::DH(dh.a, dh.alpha, dh.d, dh.theta + dh.offset);
    }
    else {
        log_ptr_->error("Unknown DH convention! Using Modified DH as default.");

        // 默认使用改进DH
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(dh.theta));
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));
        return frame;
    }
   
    
    
}

void DHParamsLoader::printFrame(const KDL::Frame& frame) const {
    // 1. 位置 (x,y,z)
    log_ptr_->info("Frame Position [m]: x = {:.4f}, y = {:.4f}, z = {:.4f}", frame.p.x(), frame.p.y(), frame.p.z());

    // 2. 旋转矩阵 (3×3)
    std::ostringstream ss;
    ss << "Frame Rotation:\n";

    for (int i = 0; i < 3; ++i)          // 行
    {
        for (int j = 0; j < 3; ++j)      // 列
            ss << std::setw(10) << frame.M(i, j) << ' ';
        ss << '\n';
    }
    log_ptr_->info(ss.str());
}

void DHParamsLoader::printDHParameters() const {
    std::ostringstream ss;
    ss << "\n=== Robot DH Parameters ===" << std::endl;
    ss << "Robot Name: " << robot_name_ << std::endl;
    ss << "Base Link: " << base_link_ << std::endl;
    ss << "Tip Link: " << tip_link_ << std::endl;
    ss << "Number of Joints: " << dh_params_.size() << std::endl;
    ss << "\nDH Parameters Table:" << std::endl;
    ss << "----------------------------------------------------------------" << std::endl;
    ss << "Joint Name           | Alpha     | a         | d         | Theta   | Offset   " << std::endl;
    ss << "----------------------------------------------------------------" << std::endl;
    
    for (const auto& dh : dh_params_) {
        ss << std::left << std::setw(20) << dh.joint_name << " | "
                  << std::fixed << std::setprecision(4)
                  << std::setw(9) << dh.alpha << " | "
                  << std::setw(9) << dh.a << " | "
                  << std::setw(9) << dh.d << " | "
                  << std::setw(9) << dh.theta << " | "
                  << std::setw(9) << dh.offset  << std::endl;
    }
    ss << "----------------------------------------------------------------" << std::endl;

    log_ptr_->info(ss.str());
}

}