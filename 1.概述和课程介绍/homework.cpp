#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include "Eigen/Dense"
using namespace std;

Eigen::Matrix3d exp_(Eigen::AngleAxisd a);
Eigen::Vector3d down_arrow(Eigen::Matrix3d A);
Eigen::Matrix3d up_arrow(Eigen::Vector3d a);
Eigen::Matrix3d exp_(Eigen::Vector3d a);

int main()
{
    //无穷小量w
    Eigen::Vector3d w(0.01,0.02,0.03);
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d (1,0,0));

    cout.precision(5);

    //求旋转向量对应的旋转矩阵和四元数
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix();
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);   //赋值为(w,x,y,z), coeffs输出为(x,y,z,w)
    cout<<q.coeffs()<<endl;
    Eigen::Quaterniond q1(1,0,0,0);
    cout<<q1.toRotationMatrix()<<endl;


    //对R进行右扰动后：
    Eigen::Matrix3d R_ = R * exp_(w);
    cout<<"旋转矩阵增量后：\n"<<R_<<endl;
    Eigen::Quaterniond q_w(1, 0.5*w(0,0), 0.5*w(1,0), 0.5*w(2,0));
    Eigen::Quaterniond q_ = q*q_w;
    q_.normalize();//归一化
    cout<<"归一化后的四元数：\n"<<q_.toRotationMatrix()<<endl;

    return 0;
}

Eigen::Matrix3d up_arrow(Eigen::Vector3d a)
{
    Eigen::Matrix3d A;
    A<<0, -a(2,0), a(1,0),
       a(2,0), 0, -a(0,0),
       -a(1,0), a(0,0), 0;
    return A;
}

Eigen::Vector3d down_arrow(Eigen::Matrix3d A)
{
    Eigen::Vector3d a;
    if((A(0,0)==A(1,1)==A(2,2))&&(A(0,1)=-A(1,0))&&(A(0,2)=-A(2,0))&&(A(2,1)=-A(1,2)))
    {
        a<<A(2,1), A(0,2), A(1,0);
        return a;
    } else
    {
        std::cout<<"The Matrix is not a 3 free-degree"<<endl;
        return Eigen::Vector3d (0,0,0);
    }
}

Eigen::Matrix3d exp_(Eigen::AngleAxisd a)
{
    Eigen::Matrix3d A;
    A = cos(a.angle())*Eigen::Matrix3d::Identity() + (1 - cos(a.angle())) * a.axis() * a.axis().transpose() + sin(a.angle()) * up_arrow(a.axis());
    return A;
}

Eigen::Matrix3d exp_(Eigen::Vector3d a)
{
    double thita = a.norm();
    Eigen::Vector3d a_ = a / thita;
    Eigen::Matrix3d A;
    A = cos(thita)*Eigen::Matrix3d::Identity() + (1-cos(thita))*a_*a_.transpose() + sin(thita)*up_arrow(a_);
    return A;
}


