#include <iostream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>

int main(int argc, char* argv[])
{
    std::cout << "Yo" << std::endl;

    nlohmann::json jsonTest;
    jsonTest["notYo"] = "definitely not yo";
    std::cout << jsonTest["notYo"] << std::endl;

    Eigen::Vector<double, 3> eigenTest = {1, 2, 4};
    std::cout << eigenTest.transpose() << std::endl;
}