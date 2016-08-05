cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


add_library(HydraGameController SHARED HydraGameController.cc)

add_library(SpawnLiquid SHARED SpawnLiquid.cc)

add_library(CreatePancake SHARED CreatePancake.cc)

add_library(ArrangeModels SHARED ArrangeModels.cc)

add_library(VisPlugin SHARED VisPlugin.cc)

add_library(SpawnUnits SHARED SpawnUnits.cc)
target_link_libraries(SpawnUnits ${GAZEBO_libraries})

add_library(HydraFreezeModels SHARED HydraFreezeModels.cc)
target_link_libraries(HydraFreezeModels ${GAZEBO_libraries})

add_library(HydraPR2RGripper SHARED HydraPR2RGripper.cc)
target_link_libraries(HydraPR2RGripper ${GAZEBO_libraries})


target_link_libraries(HydraGameController ${GAZEBO_libraries})

target_link_libraries(SpawnLiquid ${GAZEBO_libraries})

target_link_libraries(CreatePancake ${GAZEBO_libraries})

target_link_libraries(ArrangeModels ${GAZEBO_libraries} config++)

target_link_libraries(VisPlugin ${GAZEBO_libraries})
