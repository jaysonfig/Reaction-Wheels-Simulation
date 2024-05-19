#include <Utilities.hpp>

Eigen::Matrix<double, 3, 3> crs(const Eigen::Vector<double, 3> a)
{
    Eigen::Matrix<double, 3, 3> mat;
    mat <<     0, -a(2),  a(1),
            a(2),     0, -a(0),
           -a(1),  a(0),    0;
    return mat;
}

Eigen::Matrix<double, 4, 3> quaternionXiMatrix(const Eigen::Vector<double, 4> q)
{
    Eigen::Matrix<double, 4, 3> mat;
    mat.block<3, 3>(0, 0) = q(3) * Eigen::Matrix<double, 3, 3>::Identity() + crs(q.segment<3>(0));
    mat.block<1, 3>(3, 0) = -q.segment<3>(0).transpose();
    return mat;
}

void outputStates(const double simTime, const Eigen::Vector<double, 3> angularVelocity, const Eigen::Vector<double, 4> attitude, std::ofstream& output)
{
    if(output.tellp() == 0)
    {
        output << "t[s],wx[rad/s],wy[rad/s],wz[rad/s],q1,q2,q3,q4\n";
    }
    output << simTime << ",";
    for(int i = 0; i < 3; ++i)
    {
        output << angularVelocity(i) << ",";
    }
    for(int i = 0 ; i < 3; ++i)
    {
        output << attitude(i) << ",";
    }
    output << attitude(3) << "\n";
}