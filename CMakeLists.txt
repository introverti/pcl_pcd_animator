cmake_minimum_required(VERSION 3.23 FATAL_ERROR)
project(Animator)
add_compile_options(-fPIC)
set(CMAKE_CXX_STANDARD 17)
set(CXX_FLAGS "-std=c++17 -lstdc++fs")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pthread")
set(CMAKE_BUILD_TYPE "Debug")
set(PACKAGE_VERSION 1.0)

IF(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
ENDIF()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/src
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${YAML_CPP_INCLUDE_DIR}
                    $)

aux_source_directory(./src/common DIR_COM)
aux_source_directory(./src/prepare DIR_PRE)
aux_source_directory(./src/visualize DIR_VIS)

add_executable(vis ${DIR_COM} ${DIR_VIS})
target_link_libraries(vis Eigen3::Eigen ${OpenCV_LIBS} ${PCL_LIBRARIES} stdc++fs)

add_executable(pre ${DIR_COM} ${DIR_PRE})
target_link_libraries(pre Eigen3::Eigen ${PCL_LIBRARIES} stdc++fs nlohmann_json::nlohmann_json yaml-cpp)
