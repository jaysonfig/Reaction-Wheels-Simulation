#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include <Eigen/Dense>
#include <fstream>

// Eq. (2.55), pg. 29
Eigen::Matrix<double, 3, 3> crs(const Eigen::Vector<double, 3> a);

// Eq. (2.88), pg. 38
Eigen::Matrix<double, 4, 3> quaternionXiMatrix(const Eigen::Vector<double, 4> q);

void outputStates(const double simTime, 
                  const Eigen::Vector<double, 3> angularVelocity, 
                  const Eigen::Vector<double, 4> attitude, 
                  const Eigen::Vector<double, 3> wheelMomentum, 
                  const Eigen::Vector<double, -1> wheelSpinRates, 
                  std::ofstream& output);

#endif