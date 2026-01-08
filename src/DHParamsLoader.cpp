#include "rocos_app/DHParamsLoader.h"
#include <kdl/frames.hpp>

DHParamsLoader::DHParamsLoader() {
    // 初始化默认值
    robot_name_ = "Unknown";
    base_link_ = "base_link";
    tip_link_ = "tool0";
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

        std::cout << "[INFO] Loading robot: " << robot_name_ << std::endl;
        std::cout << "[INFO] Base link: " << base_link_ << std::endl;
        std::cout << "[INFO] Tip link: " << tip_link_ << std::endl;
        std::cout << "[INFO] DH Convention: " 
                  << (is_standard_dh_ ? "Standard DH" : (is_mdh_ ? "Modified DH" : "Unknown")) 
                  << std::endl;
        
        // 清空已有的DH参数
        dh_params_.clear();
        
        // 读取DH参数
        YAML::Node dh_nodes = config["dh_parameters"];
        if (!dh_nodes) {
            std::cerr << "[ERROR] No 'dh_parameters' found in YAML file!" << std::endl;
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
            std::cout << "[INFO] Loaded joint: " << dh.joint_name 
                      << " (type: " << dh.type << ")" << std::endl;
        }
        
        std::cout << "[INFO] Successfully loaded " << dh_params_.size() << " DH parameters" << std::endl;
        
        // 构建运动学链
        return buildChainFromDH();
        
    } catch (const YAML::Exception& e) {
        std::cerr << "[ERROR] Failed to load YAML file '" << yaml_file << "': " << e.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception while loading YAML: " << e.what() << std::endl;
        return false;
    }
}

bool DHParamsLoader::buildChainFromDH() {
    chain_ = KDL::Chain();
    
    if (dh_params_.empty()) {
        std::cerr << "[ERROR] No DH parameters available to build chain!" << std::endl;
        return false;
    }
    
    std::cout << "[INFO] Building kinematic chain from DH parameters..." << std::endl;
    
    // 添加基座标系
    // chain_.addSegment(KDL::Segment(base_link_ + "_base", 
    //                               KDL::Joint(KDL::Joint::None),
    //                               KDL::Frame::Identity()));
    
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
    
        
        // std::cout << "[INFO] Added joint: " << dh.joint_name 
        //           << " (a=" << dh.a << ", alpha=" << dh.alpha 
        //           << ", d=" << dh.d << ", theta=" << dh.theta 
        //           << ", offset=" << dh.offset << ")" << std::endl;
    }

    chain_.addSegment(KDL::Segment("end_effector", KDL::Joint(KDL::Joint::RotZ), KDL::Frame::Identity()));
    // 初始化关节限位
    // initializeJointLimits();
    
    std::cout << "[INFO] Successfully built kinematic chain with " 
              << chain_.getNrOfJoints() << " joints" << std::endl;
    
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
        std::cerr << "[WARNING] Unknown joint type: " << dh.type 
                  << ", using fixed joint instead." << std::endl;
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
        
        
        // std::cout << "[DEBUG] Using Modified DH transformation for joint: " << dh.joint_name << std::endl;
        double q = dh.theta + dh.offset;   // 统一先算好
        // 1. 绕X轴旋转alpha(i-1)
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));
        // 2. 沿X轴平移a(i-1)
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        // 3. 绕Z轴旋转theta(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(q ));
        // 4. 沿Z轴平移d(i)
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));
        // std::cout << "[DEBUG] Modified DH Frame for joint: " << dh.joint_name << std::endl;
        // printFrame(KDL::Frame::DH_Craig1989(dh.a, dh.alpha, dh.d, q ));
        // printFrame(frame);
        return KDL::Frame::DH_Craig1989(dh.a, dh.alpha, dh.d, q );
    } 
    else if (is_standard_dh_) {
        // 标准DH参数变换顺序 (Standard DH):
        // RotZ(theta(i)) * TransZ(d(i)) * TransX(a(i)) * RotX(alpha(i))
        
        // std::cout << "[DEBUG] Using Standard DH transformation for joint: " << dh.joint_name << std::endl;
        
        // 1. 绕Z轴旋转theta(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(dh.theta + dh.offset));
        // 2. 沿Z轴平移d(i)
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));
        // 3. 沿X轴平移a(i)
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        // 4. 绕X轴旋转alpha(i)
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));
        // std::cout << "[DEBUG] Standard DH Frame for joint: " << dh.joint_name << std::endl;
        // printFrame(KDL::Frame::DH(dh.a, dh.alpha, dh.d, dh.theta + dh.offset));

        
        // printFrame(frame);
        return KDL::Frame::DH(dh.a, dh.alpha, dh.d, dh.theta + dh.offset);
    }
    else {
        std::cerr << "[ERROR] Unknown DH convention! Using Modified DH as default." << std::endl;
        // 默认使用改进DH
        frame = frame * KDL::Frame(KDL::Rotation::RotX(dh.alpha));
        frame = frame * KDL::Frame(KDL::Vector(dh.a, 0, 0));
        frame = frame * KDL::Frame(KDL::Rotation::RotZ(dh.theta));
        frame = frame * KDL::Frame(KDL::Vector(0, 0, dh.d));
        return frame;
    }
   
    
    
}

// void DHParamsLoader::initializeJointLimits() {
//     int joint_count = chain_.getNrOfJoints();
//     q_min_.resize(joint_count);
//     q_max_.resize(joint_count);
    
//     // 初始化默认限位
//     for (int i = 0; i < joint_count; ++i) {
//         q_min_(i) = -KDL::PI;
//         q_max_(i) = KDL::PI;
//     }
    
//     // 根据DH参数设置具体限位
//     for (size_t i = 0; i < dh_params_.size() && i < static_cast<size_t>(joint_count); ++i) {
//         if (dh_params_[i].limits.size() >= 2) {
//             q_min_(i) = dh_params_[i].limits[0];
//             q_max_(i) = dh_params_[i].limits[1];
//         }
//     }
    
//     std::cout << "[INFO] Initialized joint limits for " << joint_count << " joints" << std::endl;
// }
void DHParamsLoader::printFrame(const KDL::Frame& frame) const {
    // 1. 位置 (x,y,z)
    std::cout << "位置 [m]:\n"
              << std::fixed << std::setprecision(4)
              << "  x = " << frame.p.x() << '\n'
              << "  y = " << frame.p.y() << '\n'
              << "  z = " << frame.p.z() << "\n\n";

    // 2. 旋转矩阵 (3×3)
    std::cout << "旋转矩阵:\n";
    for (int i = 0; i < 3; ++i)          // 行
    {
        for (int j = 0; j < 3; ++j)      // 列
            std::cout << std::setw(10) << frame.M(i, j) << ' ';
        std::cout << '\n';
    }
}

void DHParamsLoader::printDHParameters() const {
    std::cout << "\n=== Robot DH Parameters ===" << std::endl;
    std::cout << "Robot Name: " << robot_name_ << std::endl;
    std::cout << "Base Link: " << base_link_ << std::endl;
    std::cout << "Tip Link: " << tip_link_ << std::endl;
    std::cout << "Number of Joints: " << dh_params_.size() << std::endl;
    std::cout << "\nDH Parameters Table:" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "Joint Name           | Alpha     | a         | d         | Theta   | Offset   " << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    
    for (const auto& dh : dh_params_) {
        std::cout << std::left << std::setw(20) << dh.joint_name << " | "
                  << std::fixed << std::setprecision(4)
                  << std::setw(9) << dh.alpha << " | "
                  << std::setw(9) << dh.a << " | "
                  << std::setw(9) << dh.d << " | "
                  << std::setw(9) << dh.theta << " | "
                  << std::setw(9) << dh.offset  << std::endl;
    }
    std::cout << "----------------------------------------------------------------" << std::endl;
}