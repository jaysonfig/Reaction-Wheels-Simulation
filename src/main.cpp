#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <Vehicle.hpp>

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

    // Initialize vehicle
    Vehicle vehicle(configData["Vehicle"]);
    vehicle.setEpoch(startTime);

    // Output setup
    std::ofstream output("output/states.csv");

    for(double simTime = startTime; simTime < endTime; simTime += timeStep)
    {
        // Output states
        outputStates(simTime, vehicle.getAngularVelocity(), vehicle.getAttitude(), vehicle.getWheelMomentum(), vehicle.getWheelSpinRates(),output);

        // Gather current states
        Eigen::Vector<double, 3> externalTorque = {0, 0, 0};
        Eigen::Vector<double, 3> wheelTorque = {0, 0, 0};
        Eigen::Vector<double, 3> angularVelocity = vehicle.getAngularVelocity();
        Eigen::Vector<double, 4> attitude = vehicle.getAttitude();
        Eigen::Vector<double, 3> wheelMomentum = vehicle.getWheelMomentum();

        // Compute Derivatives
        // Eq. (3.147), pg. 102
        Eigen::Vector<double, 3> angularAcceleration = vehicle.computeGyrostatDynamics(externalTorque, wheelTorque, angularVelocity, wheelMomentum);
        // Eq. (3.20), pg. 71
        Eigen::Vector<double, 4> attitudeKinematics = vehicle.computeAttitudeKinematics(attitude, angularVelocity);

        // Euler Integration
        angularVelocity += (angularAcceleration * timeStep);
        attitude += (attitudeKinematics * timeStep);
        attitude.normalize();
        wheelMomentum += (wheelTorque * timeStep);

        // Update Vehicle
        vehicle.setAngularVelocity(angularVelocity);
        vehicle.setAttitude(attitude);
        vehicle.setWheelMomentum(wheelMomentum);
        vehicle.setEpoch(simTime + timeStep);
    }

    // Output final states and close
    // outputStates(endTime, vehicle.getAngularVelocity(), vehicle.getAttitude(), output);
    output.close();
}