#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

// Eq. (2.55), pg. 29
Eigen::Matrix<double, 3, 3> crs(const Eigen::Vector<double, 3> a)
{
    Eigen::Matrix<double, 3, 3> mat;
    mat <<     0, -a(2),  a(1),
            a(2),     0, -a(0),
           -a(1),  a(0),    0;
    return mat;
}

// Eq. (2.88), pg. 38
Eigen::Matrix<double, 4, 3> quaternionXiMatrix(const Eigen::Vector<double, 4> q)
{
    Eigen::Matrix<double, 4, 3> mat;
    mat.block<3, 3>(0, 0) = q(3) * Eigen::Matrix<double, 3, 3>::Identity() + crs(q.segment<3>(0));
    mat.block<1, 3>(3, 0) = q.segment<3>(0).transpose();
    return mat;
}

void outputStates(const double simTime, const Eigen::Vector<double, 3> angularVelocity, const Eigen::Vector<double, 4> attitude, std::ofstream& output)
{
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

int main(int argc, char* argv[])
{
    // Get config file inputs
    std::ifstream configFile("config/config.json");
    nlohmann::json configData;
    configFile >> configData;
    configFile.close();

    // Intialize timings
    const double startTime = configData["Simulation"]["startTime"];
    const double endTime = configData["Simulation"]["endTime"];
    const double timeStep = configData["Simulation"]["timeStep"];

    // Initialize Vehicle
    Eigen::Vector<double, 3> angularVelocity;
    for(int i = 0; i < 3; ++i)
    {
        angularVelocity(i) = configData["Vehicle"]["initialAngularVelocity"][i];
    }

    Eigen::Vector<double, 4> attitude;
    for(int i = 0; i < 4; ++i)
    {
        attitude(i) = configData["Vehicle"]["initialAttitude"][i];
    }

    Eigen::Matrix<double, 3, 3> inertia;
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            inertia(i, j) = configData["Vehicle"]["inertia"][i][j];
        }
    }
    Eigen::Matrix<double, 3, 3> inverseInertia = inertia.inverse();

    // Output setup
    std::ofstream output("output/states.csv");
    output << "t,wx,wy,wz,q1,q2,q3,q4\n";

    for(double simTime = startTime; simTime < endTime; simTime += timeStep)
    {
        // Output states
        outputStates(simTime, angularVelocity, attitude, output);

        Eigen::Vector<double, 3> externalTorque = {0, 0, 0};
        // Eq. (3.81), pg. 84
        Eigen::Vector<double, 3> angularAcceleration = inverseInertia * (externalTorque - crs(angularVelocity) * inertia * angularVelocity);
        // Eq. (3.20), pg. 71
        Eigen::Vector<double, 4> attitudeKinematics = 0.5 * quaternionXiMatrix(attitude) * angularVelocity;

        // Euler Integration
        angularVelocity += (angularAcceleration * timeStep);
        attitude += (attitudeKinematics * timeStep);
        attitude.normalize();
    }

    // Output final states and close
    outputStates(endTime, angularVelocity, attitude, output);
    output.close();
}