#include <Vehicle.hpp>

Vehicle::Vehicle()
{
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
                                                                 Eigen::Vector<double, 3> angularVelocity)
{
    return this->inverseInertia * (externalTorques - crs(angularVelocity) * this->inertia * angularVelocity);
}

const Eigen::Vector<double, 4> Vehicle::computeAttitudeKinematics(Eigen::Vector<double, 4> attitude,
                                                                  Eigen::Vector<double, 3> angularVelocity)
{
    return 0.5 * quaternionXiMatrix(attitude) * angularVelocity;
}

const Eigen::Vector<double, 3> Vehicle::computeGyrostatDynamics(Eigen::Vector<double, 3> externalTorques,
                                                                Eigen::Vector<double, 3> internalTorques,
                                                                Eigen::Vector<double, 3> angularVelocity,
                                                                Eigen::Vector<double, 3> wheelAngularMomentum)
{
    return this->inverseInertia * (externalTorques - internalTorques - crs(angularVelocity) * (this->inertia * angularVelocity + wheelAngularMomentum));
}