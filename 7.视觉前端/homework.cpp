/*  your code start  */
    int size = end_frame_id - start_frame_id;
    MatXX D(MatXX::Zero(2*size, 4));
    TicToc tictoc;

    /// ************************构建D矩阵************************** ///
    for (int j = start_frame_id; j < end_frame_id; ++j)
    {
        double u = camera_pose[j].uv.x();
        double v = camera_pose[j].uv.y();
        Eigen::Matrix<double, 1, 4> Pj3,Pj2,Pj1,pj1,pj2;

        Eigen::Matrix<double,3,4> P(Eigen::Matrix<double,3,4>::Zero());
        P.leftCols<3>() = camera_pose[j].Rwc.transpose();
        P.rightCols<1>() = -camera_pose[j].Rwc.transpose() * camera_pose[j].twc;

        Pj1 = P.block(0,0,1,4);
        Pj2 = P.block(1,0,1,4);
        Pj3 = P.block(2,0,1,4);

        pj1 = u * Pj3 - Pj1;
        pj2 = v * Pj3 - Pj2;

        D.block(2*(j-start_frame_id)  , 0,1,4) = pj1;
        D.block(2*(j-start_frame_id)+1, 0,1,4) = pj2;
    }
    double time = tictoc.toc();
    cout<<"构建D矩阵耗时 "<<time<<"s"<<endl;


    /// *************************对D进行缩放************************* ///
    double max_D = D.maxCoeff();
    MatXX S = Eigen::MatrixXd::Identity(4,4) / max_D;
    D = D * S;



    /// **************************判断此次三角化是否有效************************ ///
    MatXX DTD;
    DTD = D.transpose() * D;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd (DTD, Eigen::ComputeFullU | Eigen::ComputeFullV);
    cout<<"U is: "<<endl<<svd.matrixU()<<endl;
    cout<<"V is: "<<endl<<svd.matrixV()<<endl;
    cout<<"eigenvalue is: "<<endl<<svd.singularValues()<<endl;
    double sigma4, sigma3;
    sigma3 = svd.singularValues()[2];
    sigma4 = svd.singularValues()[3] + 1e-16; // 防止出现除以0的情况
    if ((sigma3 / sigma4) > 1e10)
        cout<<"======================"<<endl<<"the result is reliable!"<<endl<<"======================"<<endl;
    else
        cout<<"======================"<<endl<<"the result is invalid!"<<endl<<"======================"<<endl;
    Eigen::Vector4d u4;
    u4 = svd.matrixU().block(0,3,4,1);
    P_est = Eigen::Vector3d(u4[0]/u4[3], u4[1]/u4[3], u4[2]/u4[3]);


    /// *************************对y投影的正负号进行判定************************* ///
    if (P_est.z()>0)
        cout<<"The point is in front of camera!"<<endl<<"======================"<<endl;


    double time2 = tictoc.toc();
    cout<<"完成三角化耗时"<<time2<<"s"<<endl;
/* your code end */
