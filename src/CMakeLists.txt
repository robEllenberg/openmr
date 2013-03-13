cmake_minimum_required (VERSION 2.6.0)
project (openmr)

option(BUILD_ACHCONTROLLER "Build achcontroller (requires ach libraries)" ON)
option(USE_CUSTOM_ODERAVE "Use Rob's custom ODE plugin (only if you know what you're doing!)" OFF)

find_package(OpenRAVE REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall -std=gnu++0x")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

if( NOT OpenRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OpenRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(PLUGIN_LINK_FLAGS "")
endif()

include_directories(${OpenRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/../include")

link_directories(${OpenRAVE_LINK_DIRS})
link_directories(${Boost_LIBRARY_DIRS}) 

add_library(openmr SHARED openmrmain.cpp)
set_target_properties(openmr PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}")
set_target_properties(openmr PROPERTIES LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")

if( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../plugins")
else( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )
    set(PLUGIN_INSTALL_DIR "$ENV{PLUGIN_INSTALL_DIR}")
endif( NOT DEFINED ENV{PLUGIN_INSTALL_DIR} )

find_library(ACH_DIR ach)

if(ACH_DIR)
    message(STATUS "Found ACH libraries, building achcontroller")
else()
    set(BUILD_ACH OFF)
endif()

target_link_libraries(openmr ${OpenRAVE_LIBRARIES})
install(TARGETS openmr DESTINATION ${PLUGIN_INSTALL_DIR})

if(ACH_DIR AND BUILD_ACHCONTROLLER)
    add_definitions(-DBUILD_ACHCONTROLLER)
    add_executable(demo demo-loop.cpp)
    target_link_libraries(openmr ${OpenRAVE_LIBRARIES} ach)
    target_link_libraries(demo ${OpenRAVE_LIBRARIES} ach)
endif()

if(USE_CUSTOM_ODERAVE)
    add_definitions(-DUSE_CUSTOM_ODE_PLUGIN)
endif()

