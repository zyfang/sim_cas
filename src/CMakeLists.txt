cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_library(HydraGameController SHARED HydraGameController.cc)

add_library(HydraGameControllerJs SHARED HydraGameControllerJs.cc)


add_library(SpawnLiquid SHARED SpawnLiquid.cc)

add_library(CreatePancake SHARED CreatePancake.cc)



target_link_libraries(HydraGameController ${GAZEBO_libraries})

target_link_libraries(HydraGameControllerJs ${GAZEBO_libraries})

target_link_libraries(SpawnLiquid ${GAZEBO_libraries})

target_link_libraries(CreatePancake ${GAZEBO_libraries})
