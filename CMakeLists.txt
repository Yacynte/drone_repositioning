cmake_minimum_required(VERSION 3.10)
project(ImageMatcher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include )

# message(STATUS ${OpenCV_INCLUDE_DIRS})

add_executable(ImageMatcher
    src/main.cpp
    src/ImageMatcher.cpp
    src/MetadataClient.cpp
    src/RstpReader.cpp
)

target_link_libraries(ImageMatcher ${OpenCV_LIBS})
