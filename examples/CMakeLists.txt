cmake_minimum_required (VERSION 2.4)
project (openrave-modular-robots-examples)

include(FindOpenRAVE.cmake)

if( NOT OPENRAVE_FOUND )
  message(FATAL_ERROR "could not find openrave, check for openrave-config in path")
endif( NOT OPENRAVE_FOUND )

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

include_directories(${OPENRAVE_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
link_directories(${OPENRAVE_LINK_DIRS})

add_executable(Test-servocontroller-1 Test-servocontroller-1.cpp)
set_target_properties(Test-servocontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Test-sinoscontroller-1 Test-sinoscontroller-1.cpp)
set_target_properties(Test-sinoscontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-sinoscontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-sinoscontroller-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})






