cmake_minimum_required(VERSION 3.5)

project(udp_cam_receiver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    image_transport
    roscpp
    cv_bridge
)

catkin_package()

find_package(PkgConfig)
pkg_check_modules(ffmpeg REQUIRED libavcodec libavutil libswscale IMPORTED_TARGET)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(udp_cam_receiver
    udp_cam_receiver.cpp
    udp_socket.cpp
)

target_link_libraries(udp_cam_receiver
    ${catkin_LIBRARIES}
    PkgConfig::ffmpeg
)

install(TARGETS udp_cam_receiver
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

