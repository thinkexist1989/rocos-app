//
// Created by think on 2022/2/20.
//

#include "kinematics.hpp"

#include <kdl_parser/kdl_parser.hpp>  // 用于将urdf文件解析为KDL::Tree

#include "trac_ik/trac_ik.hpp"  //逆运动学处理

namespace rocos {

    Kinematics::Kinematics() {

    }

    Kinematics::~Kinematics() {

    }

    bool Kinematics::setChain(const Chain &chain) {
        chain_ = chain;

        q_min_.resize(chain_.getNrOfJoints());
        
        q_max_.resize(chain_.getNrOfJoints());
//        Initialize(); // 不要在这里初始化，要等q_min，q_max都初始化完毕再初始化

        return true;
    }

    bool Kinematics::setChain(const Tree &tree, const std::string &base_link, const std::string &tip) {
        tree_ = tree;

        if(!tree_.getChain(base_link, tip, chain_)) {
            // 从KDL::Tree获取运动链失败
            std::cerr << "[ERROR][rocos::Kinematics] Could not get chain from kdl tree!" << std::endl;
            return false;
        }

        q_min_.resize(chain_.getNrOfJoints());
        q_max_.resize(chain_.getNrOfJoints());

        return true;
    }

    void Kinematics::Initialize() {
        // 初始化正运动学求解器
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        //初始化逆运动学求解器
        ik_solver_.reset(new TRAC_IK::TRAC_IK(chain_, q_min_, q_max_ ,0.005,7e-7));
    }

    int Kinematics::JntToCart(const JntArray &q_in, Frame &p_out) {
        return fk_solver_->JntToCart(q_in, p_out);
    }

    int Kinematics::CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out) {
        return ik_solver_->CartToJnt(q_init, p_in, q_out);
    }

    bool Kinematics::setPosLimits(const JntArray &q_min, const JntArray &q_max) {
        q_min_ = q_min;
        q_max_ = q_max;

        return true;
    }

}