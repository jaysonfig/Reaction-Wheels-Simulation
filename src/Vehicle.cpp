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
            this->inertiaWithoutWheels(i, j) = vehicleConfig["inertiaWithoutWheels"][i][j];
        }
    }

    this->numWheels = vehicleConfig["numWheels"];
    this->wheelDistribution.resize(3, numWheels);
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < numWheels; ++j)
        {
            this->wheelDistribution(i,j) = vehicleConfig["wheelDistribution"][i][j];
        }
    }
    this->pInverseWheelDistribution = wheelDistribution.completeOrthogonalDecomposition().pseudoInverse();
    for(int i = 0; i < 3; ++i)
    {
        this->wheelMomentum(i) = vehicleConfig["wheelMomentum"][i];
    }
    this->wheelSpinInertia = vehicleConfig["wheelSpinInertia"];
    this->wheelPerpendicularInertia = vehicleConfig["wheelPerpendicularInertia"];

    this->inertia = this->inertiaWithoutWheels;
    for(int i = 0; i < numWheels; ++i)
    {
        this->inertia += (this->wheelPerpendicularInertia * (Eigen::Matrix<double, 3, 3>::Identity() - this->wheelDistribution.col(i) * this->wheelDistribution.col(i).transpose()));
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
                                                                Eigen::Vector<double, 3> wheelTorques,
                                                                Eigen::Vector<double, 3> angularVelocity,
                                                                Eigen::Vector<double, 3> wheelAngularMomentum)
{
    return this->inverseInertia * (externalTorques - wheelTorques - crs(angularVelocity) * (this->inertia * angularVelocity + wheelAngularMomentum));
}

const Eigen::Vector<double, -1> Vehicle::getWheelSpinRates()
{
    Eigen::Vector<double, -1> wheelMomentumInWheelSpace = pInverseWheelDistribution * wheelMomentum;
    return wheelMomentumInWheelSpace / wheelSpinInertia - wheelDistribution.transpose() * angularVelocity;
}