#include <gtest/gtest.h>
#include <Utilities.hpp>
#include <sstream>
#include <cstdio>

TEST(UtilityTests, crs1)
{
    Eigen::Vector<double, 3> a(1, 2, 3);
    Eigen::Matrix<double, 3, 3> expected;
    expected << 0, -3, 2,
                3, 0, -1,
                -2, 1, 0;

    Eigen::Matrix<double, 3, 3> result = crs(a);
    EXPECT_EQ(result, expected);
}

TEST(UtilityTests, quaternionXiMatrix1)
{
    Eigen::Vector<double, 4> q(1, 2, 3, 4);
    Eigen::Matrix<double, 4, 3> expected;
    expected << 4, -3, 2,
                3, 4, -1,
                -2, 1, 4,
                -1, -2, -3;
    
    Eigen::Matrix<double, 4, 3> result = quaternionXiMatrix(q);
    EXPECT_EQ(result, expected);
}

TEST(UtilityTests, outputStates)
{    
    std::ofstream out("output_test.csv");

    double simTime = 2;
    Eigen::Vector<double, 3> w(3, 4, 5);
    Eigen::Vector<double, 4> q(6, 7, 8, 9);

    std::string expected =
        "t[s],wx[rad/s],wy[rad/s],wz[rad/s],q1,q2,q3,q4\n2,3,4,5,6,7,8,9\n2,3,4,5,6,7,8,9\n";

    outputStates(simTime, w, q, out);
    outputStates(simTime, w, q, out);

    out.close();
    std::ifstream in("output_test.csv");
    std::stringstream buffer;
    buffer << in.rdbuf();
    in.close();

    EXPECT_EQ(buffer.str(), expected);
    std::remove("output_test.csv");
}
