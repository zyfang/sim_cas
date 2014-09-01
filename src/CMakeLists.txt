cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_library(HydraGameController SHARED HydraGameController.cc)

target_link_libraries(
	HydraGameController
    ${GAZEBO_libraries}
)
