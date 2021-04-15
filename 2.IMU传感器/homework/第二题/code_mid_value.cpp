/读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i)
    {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();

        /// imu 动力学模型 欧拉积分
        // 我不认为这里应该反一下，k+1时刻的值都是用k时刻的来积分出来的
        // 那么所以a也应该用k时刻的
        // 初始值并没有被放进去，说明初始值是为了计算i=1的pvq而存在的
        // 或者可以理解为是用初始值在dt上的积分来代替第一个值进行存储
        // 所以这个时候的a应该是用初始值的q
//        Qwb = Qwb * dq;
//        Qwb = Qwb.normalize();
//        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
//        Vw = Vw + acc_w * dt;
//        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;

        /// 中值积分
        /// 我这里用k与k+1的值来作为k时刻的值
        MotionData imupose_next = imudata[i];
        if (i != (imudata.size()-1))
            imupose_next = imudata[i+1];// 如果不是最后一个，就正常处理：k+1代表下一个
        Eigen::Quaterniond dq_mid;
        Eigen::Vector3d dtheta_half_mid = (imupose.imu_gyro + imupose_next.imu_gyro) * dt / 4.0;
        dq_mid.w() = 1;
        dq_mid.x() = dtheta_half_mid.x();
        dq_mid.y() = dtheta_half_mid.y();
        dq_mid.z() = dtheta_half_mid.z();

        Eigen::Vector3d acc_w_mid = (Qwb * imupose.imu_acc + Qwb * dq_mid * imupose_next.imu_acc) / 2.0 + gw;
        Vw = Vw + acc_w_mid * dt;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w_mid;
        Qwb = Qwb * dq_mid;
		Qwb = Qwb.normalize();


        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }
