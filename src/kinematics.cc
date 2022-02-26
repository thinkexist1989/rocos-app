//
// Created by think on 2022/2/20.
//

#include "kinematics.h"


namespace rocos {

    Kinematics::Kinematics() {

        //TODO: 初始化泰科6-DOF机械臂
        getChainTechServo(chain_, q_min_, q_max_);
        //TODO: 初始化7-DOF机械臂
//        getChain7Dofs(chain_, q_min_, q_max_);

        Initialize();
    }

    Kinematics::~Kinematics() {

    }

    void Kinematics::Initialize() {
        // 初始化正运动学求解器
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        //初始化逆运动学求解器
        ik_solver_.reset(new TRAC_IK::TRAC_IK(chain_, q_min_, q_max_));
    }

    void Kinematics::getChainTechServo(KDL::Chain &chain, KDL::JntArray &q_min, KDL::JntArray &q_max) {
        KDL::Chain kin_chain;
        q_min.resize(6);
        q_max.resize(6);
        //joint 0
//        kin_chain.addSegment(Segment(Joint(Joint::None), Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0)));
        //joint 1
        kin_chain.addSegment(Segment(Joint(Joint::None), Frame::DH_Craig1989(0.0, 0.0, 0.1215, 0.0)));
        q_min(0) = -M_PI ; q_max(0) = M_PI; //泰科手册里1关节运动范围±360，暂时保守点，不知道驱动器带不带限位保护
        //joint 2
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0, M_PI_2, 0.1225, M_PI_2)));
        q_min(1) = -M_PI ; q_max(1) = M_PI;
        //joint 3
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.25, M_PI, 0.0, 0.0)));
        q_min(2) = -M_PI ; q_max(2) = M_PI;
        //joint 4
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.21, M_PI, -0.012, -M_PI_2)));
        q_min(3) = -M_PI ; q_max(3) = M_PI;
        //joint 5
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, -M_PI_2, 0.090, 0.0)));
        q_min(4) = -M_PI ; q_max(4) = M_PI;
        //joint 6
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, M_PI_2, 0.088, 0.0)));
        q_min(5) = -M_PI ; q_max(5) = M_PI;
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0)));

        chain = kin_chain;
    }

    void Kinematics::getChain7Dofs(KDL::Chain &chain, KDL::JntArray &q_min, KDL::JntArray &q_max) {
        KDL::Chain kin_chain;
        q_min.resize(7);
        q_max.resize(7);
        //joint 0
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, 0.0, 0.3393, 0.0)));
        q_min(0) = -168.0 / 180.0 * M_PI ; q_max(0) = 168.0 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 1
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
        q_min(1) = -130.5 / 180.0 * M_PI ; q_max(1) = 130.5 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 2
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, -M_PI_2, 0.3977, 0.0)));
        q_min(2) = -159.5 / 180.0 * M_PI ; q_max(2) = 159.5 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 3
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
        q_min(3) = -131.5 / 180.0 * M_PI ; q_max(3) = 131.5 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 4
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, -M_PI_2, 0.3925, 0.0)));
        q_min(4) = -159.0 / 180.0 * M_PI ; q_max(4) = 159.0 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 5
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, M_PI_2, 0.0, 0.0)));
        q_min(5) = -124.5 / 180.0 * M_PI ; q_max(5) = 124.5 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 6
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, -M_PI_2, 0.2145, 0.0)));
        q_min(5) = -167.5 / 180.0 * M_PI ; q_max(6) = 167.5 / 180.0 * M_PI; //见调试总结，关节限位值
        //joint 7
        kin_chain.addSegment(Segment(Joint(Joint::Fixed), Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0)));


        chain = kin_chain;
    }

    int Kinematics::JntToCart(const JntArray &q_in, Frame &p_out) {
        return fk_solver_->JntToCart(q_in, p_out);
    }

    int Kinematics::CartToJnt(const JntArray &q_init, const Frame &p_in, JntArray &q_out) {
        return ik_solver_->CartToJnt(q_init, p_in, q_out);
    }

}