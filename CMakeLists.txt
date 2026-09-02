cmake_minimum_required(VERSION 3.10)
project(ImageMatcher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include )

# message(STATUS ${OpenCV_INCLUDE_DIRS})

add_executable(ImageMatcher
    src_cpp/main.cpp
    src_cpp/ImageMatcher.cpp
    src_cpp/MetadataClient.cpp
    src_cpp/RstpReader.cpp
    src_cpp/Utils.cpp
    src_cpp/Logger.cpp
)

target_link_libraries(ImageMatcher ${OpenCV_LIBS} Eigen3::Eigen)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall")

message(STATUS "Build type is: ${CMAKE_BUILD_TYPE}")
