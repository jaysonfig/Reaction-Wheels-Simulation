#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <Utilities.hpp>

class Vehicle
{
    public:
        Vehicle();
        ~Vehicle();
        Vehicle(nlohmann::json vehicleConfig);

        const Eigen::Vector<double, 3> computeRigidBodyDynamics(Eigen::Vector<double, 3> externalTorques, 
                                                                Eigen::Vector<double, 3> angularVelocity);

        const Eigen::Vector<double, 4> computeAttitudeKinematics(Eigen::Vector<double, 4> attitude,
                                                                 Eigen::Vector<double, 3> angularVelocity);

        const Eigen::Vector<double, 3> computeGyrostatDynamics(Eigen::Vector<double, 3> externalTorques,
                                                               Eigen::Vector<double, 3> internalTorques,
                                                               Eigen::Vector<double, 3> angularVelocity,
                                                               Eigen::Vector<double, 3> wheelAngularMomentum);

        inline const double getEpoch()
        {
            return epoch;
        }

        inline const Eigen::Vector<double, 3> getAngularVelocity()
        {
            return angularVelocity;
        };

        inline const Eigen::Vector<double, 4> getAttitude()
        {
            return attitude;
        };

        inline const Eigen::Matrix<double, 3, 3> getInertia()
        {
            return inertia;
        };

        inline const Eigen::Matrix<double, 3, 3> getInverseInertia()
        {
            return inverseInertia;
        };

        inline void setEpoch(const double epoch)
        {
            this->epoch = epoch;
        }

        inline void setAngularVelocity(const Eigen::Vector<double, 3> angularVelocity)
        {
            this->angularVelocity = angularVelocity;
        };

        inline void setAttitude(const Eigen::Vector<double, 4> attitude)
        {
            this->attitude = attitude;
        };

    private:
        double epoch;
        Eigen::Vector<double, 3> angularVelocity;
        Eigen::Vector<double, 4> attitude;
        Eigen::Matrix<double, 3, 3> inertia;
        Eigen::Matrix<double, 3, 3> inverseInertia;
};

#endif