#include <Vehicle.hpp>

Vehicle::Vehicle()
{
    this->epoch = -1;
    this->angularVelocity = {-1, -1, -1};
    this->attitude = {-1, -1, -1, -1};
    this->inertia << -1, -1, -1, 
                     -1, -1, -1, 
                     -1, -1, -1;
    this->inverseInertia << -1, -1, -1, 
                            -1, -1, -1, 
                            -1, -1, -1;
}

Vehicle::~Vehicle()
{

}

Vehicle::Vehicle(nlohmann::json vehicleConfig)
{
    this->epoch = -1;
    for(int i = 0; i < 3; ++i)
    {
        this->angularVelocity(i) = vehicleConfig["angularVelocity"][i];
    }
    for(int i = 0; i < 4; ++i)
    {
        this->attitude(i) = vehicleConfig["attitude"][i];
    }
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            this->inertia(i, j) = vehicleConfig["inertia"][i][j];
        }
    }
    this->inverseInertia = this->inertia.inverse();
}

const Eigen::Vector<double, 3> Vehicle::computeRigidBodyDynamics(Eigen::Vector<double, 3> externalTorques, 
                                                                 Eigen::Vector<double, 3> angularVelocity, 
                                                                 Eigen::Matrix<double, 3, 3> inertia, 
                                                                 Eigen::Matrix<double, 3, 3> inverseInertia)
{
    return inverseInertia * (externalTorques - crs(angularVelocity) * inertia * angularVelocity);
}

const Eigen::Vector<double, 3> Vehicle::computeRigidBodyDynamics(Eigen::Vector<double, 3> externalTorques, 
                                                                 Eigen::Vector<double, 3> angularVelocity, 
                                                                 Eigen::Matrix<double, 3, 3> inertia)
{
    Eigen::Matrix<double, 3, 3> inverseInertia = inertia.inverse();
    return this->computeRigidBodyDynamics(externalTorques, angularVelocity, inertia, inverseInertia);
}

const Eigen::Vector<double, 3> Vehicle::computeRigidBodyDynamics(Eigen::Vector<double, 3> externalTorques, 
                                                                 Eigen::Vector<double, 3> angularVelocity)
{
    return this->computeRigidBodyDynamics(externalTorques, angularVelocity, this->inertia, this->inverseInertia);
}

const Eigen::Vector<double, 4> Vehicle::computeAttitudeKinematics(Eigen::Vector<double, 4> attitude,
                                                                  Eigen::Vector<double, 3> angularVelocity)
{
    return 0.5 * quaternionXiMatrix(attitude) * angularVelocity;
}