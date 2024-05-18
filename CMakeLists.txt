cmake_minimum_required(VERSION 3.1)
project(ReactionWheels LANGUAGES CXX)

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)

include_directories(include)

add_executable(ReactionWheels 
                    # Main
                    src/main.cpp
                    # source
                    src/Vehicle.cpp
                    src/Utilities.cpp
                    
                    # include
                    # include/Vehicle.hpp
                    # include/Utilities.hpp
                )

target_link_libraries(ReactionWheels PRIVATE nlohmann_json::nlohmann_json Eigen3::Eigen)