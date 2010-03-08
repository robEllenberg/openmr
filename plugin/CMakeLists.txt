cmake_minimum_required (VERSION 2.4)
project (openrave-modular-robots)

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${OPENRAVE_LINK_DIRS})

add_library(mrcontrollers SHARED mrcontrollersmain.cpp servocontroller.h plugindefs.h)
target_link_libraries(mrcontrollers ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

include_directories(mrcontrollers "${CMAKE_SOURCE_DIR}")
set_target_properties(mrcontrollers PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}"
                                      LINK_FLAGS "${PLUGIN_LINK_FLAGS}")
install(TARGETS mrcontrollers DESTINATION /home/juan/desarrollo/openrave/localinstall/share/openrave/plugins )



