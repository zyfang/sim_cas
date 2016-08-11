cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sim_cas)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")	

#set the default path for built executables to the "build" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build)

## Find Gazebo package
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  #pkg_check_modules(GAZEBO gazebo)
  find_package(gazebo REQUIRED)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()


include_directories(
	${GAZEBO_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
	${OGRE-Terrain_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
)

link_directories(
	${GAZEBO_LIBRARY_DIRS}
)

add_subdirectory(src)

