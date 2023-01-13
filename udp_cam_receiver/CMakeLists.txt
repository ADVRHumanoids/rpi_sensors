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

include_directories(${catkin_INCLUDE_DIRS})

find_library(x264_LIBRARY x264)

if(NOT x264_LIBRARY)
    message(WARNING "x264 library not found, not building senders...")
endif()

find_library(LIBAVUTIL_LIBRARY avutil HINTS /opt/ffmpeg/lib)
find_library(LIBAVCODEC_LIBRARY avcodec HINTS /opt/ffmpeg/lib)
find_library(LIBSWSCALE_LIBRARY swscale HINTS /opt/ffmpeg/lib)

set(FFMPEG_INCLUDE_PATH "/opt/ffmpeg/include" CACHE PATH "path to libavcodec/avcodec.h")
include_directories(${FFMPEG_INCLUDE_PATH})

if(NOT LIBAVCODEC_LIBRARY OR NOT FFMPEG_INCLUDE_PATH)
    message(WARNING "Could not find ffmpeg libraries, not building receivers...")
endif()


if(LIBAVCODEC_LIBRARY AND EXISTS ${FFMPEG_INCLUDE_PATH})
    
    add_executable(udp_cam_receiver
        udp_cam_receiver.cpp
        udp_socket.cpp
    )

    target_link_libraries(udp_cam_receiver
        ${catkin_LIBRARIES}
        ${LIBAVUTIL_LIBRARY}
        ${LIBAVCODEC_LIBRARY}
        ${LIBSWSCALE_LIBRARY}
    )

    install(TARGETS udp_cam_receiver
        DESTINATION ${CATKIN_PACKAGE_BIN_DIR}
    )
else()
    message(FATAL "libavcodec not found")
endif()
