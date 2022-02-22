cmake_minimum_required(VERSION 3.0)
add_definitions(-D_MACOS)

add_library(SDK_LIBRARY SHARED 
    src/rplidar_driver.cpp
    src/sl_crc.cpp
    src/sl_lidar_driver.cpp
    src/sl_serial_channel.cpp
    src/sl_tcp_channel.cpp
    src/sl_udp_channel.cpp
    
    src/hal/thread.cpp

    src/arch/macOS/net_serial.cpp
    src/arch/macOS/net_socket.cpp
    src/arch/macOS/timer.cpp
)

target_include_directories(SDK_LIBRARY PUBLIC 
    ${CMAKE_SOURCE_DIR}/sdk/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
    ${CMAKE_CURRENT_SOURCE_DIR}/src/arch/macOS
)

target_link_libraries(SDK_LIBRARY PUBLIC pthread)
