//
// Created by think on 2022/2/20.
//

#include <rocos_app/kinematics.h>
#include <kdl_parser/kdl_parser.hpp> // 用于将urdf文件解析为KDL::Tree


namespace rocos {

    Kinematics::Kinematics() {

        //TODO: 初始化泰科6-DOF机械臂
//        getChainTechServo(chain_, q_min_, q_max_);
        //TODO: 初始化7-DOF机械臂
//        getChain7Dofs(chain_, q_min_, q_max_);

//        Initialize(); // 初始化
    }

    Kinematics::Kinematics(const std::string &urdf_file_path,
                           const std::string &base_link,
                           const std::string &tip) {
        KDL::Tree tree;
        if(!kdl_parser::treeFromFile(urdf_file_path, tree)) {
            // 解析失败
            std::cerr << "[ERROR][rocos::Kinematics] Could not extract urdf to kdl tree!" << std::endl;
            return;
        }

        tree.getChain(base_link, tip, chain_); // 从KDL::Tree获取运动链

//        Initialize(); // 初始化
    }

    Kinematics::Kinematics(const KDL::Chain& chain) {
        setChain(chain);
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

    bool Kinematics::setChain(const std::string &base_link, const std::string &tip) {

        if(!tree_.getChain(base_link, tip, chain_)) {
            // 从KDL::Tree获取运动链失败
            std::cerr << "[ERROR][rocos::Kinematics] Could not get chain from kdl tree!" << std::endl;
            return false;
        }

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
//        Initialize(); // 不要在这里初始化，要等q_min，q_max都初始化完毕再初始化

        return true;
    }

    void Kinematics::Initialize() {
        // 初始化正运动学求解器
        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        //初始化逆运动学求解器
        ik_solver_.reset(new TRAC_IK::TRAC_IK(chain_, q_min_, q_max_ ,0.005,7e-7));
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
        kin_chain.addSegment(Segment(Joint(Joint::None), Frame::DH_Craig1989(0.0, 0.0, 0.0, 0.0)));


        chain = kin_chain;
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