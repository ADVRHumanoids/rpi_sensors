cmake_minimum_required(VERSION 3.10)
project(rpi_cam)

option(RPI "Select if building RaspberryPi modules" OFF)
option(ROBOT "Select if building robot modules" OFF)

if(${RPI})
    add_subdirectory(rpi_services)
    add_subdirectory(udp_cam_sender)
endif()

if(${ROBOT})
    add_subdirectory(udp_cam_receiver)
endif()
