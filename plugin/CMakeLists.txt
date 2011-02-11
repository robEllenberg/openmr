cmake_minimum_required (VERSION 2.4)
project (openmr)

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${OPENRAVE_LINK_DIRS})
add_library(openmr SHARED openmrmain.cpp  plugindefs.h servocontroller.h sinoscontroller.h)
set_target_properties(openmr PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                                   LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
target_link_libraries(openmr ${OPENRAVE_LIBRARY} )
install(TARGETS openmr DESTINATION share/openrave/plugins)

