cmake_minimum_required(VERSION 3.14)
project(ReactionWheelTest)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

include_directories(../include)

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

add_executable(ReactionWheelTest
                    # source
                    ../src/Utilities.cpp

                    # include
                    ../include/Utilities.hpp

                    # tests
                    unittests/Utility_test.cpp
)
target_link_libraries(
  ReactionWheelTest
  GTest::gtest_main
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

include(GoogleTest)
gtest_discover_tests(ReactionWheelTest)