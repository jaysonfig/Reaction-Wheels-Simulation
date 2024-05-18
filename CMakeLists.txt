cmake_minimum_required(VERSION 3.1)
project(ReactionWheels LANGUAGES CXX)

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

add_executable(ReactionWheels src/main.cpp)
target_link_libraries(ReactionWheels PRIVATE nlohmann_json::nlohmann_json Eigen3::Eigen)