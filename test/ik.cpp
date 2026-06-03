        int jnt_num_=robot.getJointNum();;
        KDL::Frame frame(
        KDL::Rotation::RPY(roll, pitch, yaw),  // 旋转矩阵
        KDL::Vector(x, y, z)                   // 平移向量
        );//参数可以直接flange读取Frame getFlange()，读取回来就是Frame类型
        // 2. 如果只有位置、没有旋转
        KDL::Frame frame_pos_only(KDL::Vector(x, y, z));  // 默认 R = Identity
        // 3. 如果只有旋转、没有位置
        KDL::Frame frame_rot_only(KDL::Rotation::RPY(roll, pitch, yaw));  // p = 0
        // ========== 验证输出 ==========
        std::cout << frame << std::endl;
        KDL::Frame frame;//放求解的笛卡尔姿态

        KDL::JntArray q_init(jnt_num_);//当前初始关节角
        KDL::JntArray q_target(jnt_num_);//求解出来的关节角
        for (int i = 0; i < jnt_num_; i++) {
            q_init.data[i] = pos_[i];
            q_target.data[i] = pos_[i];
        }
        if (robot.CartToJnt(q_init, pose, q_target) < 0) {
            PLOG_ERROR << " CartToJnt failed";
            return -1;
        }