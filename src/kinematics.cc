//
// Created by think on 2022/2/20.
//

#include "kinematics.h"


namespace rocos {
    using namespace KDL;

    void Kinematics::initialize() {

    }

    void Kinematics::getChain(KDL::Chain &chain, KDL::JntArray &q_min, KDL::JntArray &q_max) {
        KDL::Chain kin_chain;
        q_min.resize(6);
        q_max.resize(6);
        //joint 0
        kin_chain.addSegment(Segment(Joint(Joint::None), Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0)));
        //joint 1
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, 0.0, 0.1215, 0.0)));
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
        kin_chain.addSegment(Segment(Joint(Joint::RotZ), Frame::DH_Craig1989(0.0, M_PI, 0.088, 0.0)));
        q_min(5) = -M_PI ; q_max(5) = M_PI;

        chain = kin_chain;
    }

}