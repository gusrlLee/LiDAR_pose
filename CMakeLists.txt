cmake_minimum_required(VERSION 3.0)
project(LIDAR_pose)
find_package(OpenCV REQUIRED)
add_definitions(-D_MACOS)

add_compile_options("-g" "-Wall")

set(SDK_INCLUDE_DIRS 
    ${CMAKE_SOURCE_DIR}/sdk/include
    ${CMAKE_SOURCE_DIR}/sdk/hal

)

add_subdirectory(sdk)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC SDK_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} SDK_LIBRARY ${OpenCV_LIBS})