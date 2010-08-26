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

#-- Examples for the servocontroller

add_executable(Test-servocontroller-1 Test-servocontroller-1.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Test-servocontroller-2 Test-servocontroller-2.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-2 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-2 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-2 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Test-servocontroller-3 Test-servocontroller-3.cpp TestBase.cpp)
set_target_properties(Test-servocontroller-3 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-servocontroller-3 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-servocontroller-3 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

#-- Examples for the sinoscontroller

add_executable(Test-sinoscontroller-1 Test-sinoscontroller-1.cpp TestBase.cpp)
set_target_properties(Test-sinoscontroller-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-sinoscontroller-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-sinoscontroller-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Test-sinoscontroller-2 Test-sinoscontroller-2.cpp TestBase.cpp)
set_target_properties(Test-sinoscontroller-2 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Test-sinoscontroller-2 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Test-sinoscontroller-2 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Minicube-I-1 Minicube-I-1.cpp TestBase.cpp)
set_target_properties(Minicube-I-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Minicube-I-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Minicube-I-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Cube3-I Cube3-I.cpp TestBase.cpp)
set_target_properties(Cube3-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube3-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube3-I ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Cube4-I Cube4-I.cpp TestBase.cpp)
set_target_properties(Cube4-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube4-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube4-I ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(Cube5-I Cube5-I.cpp TestBase.cpp)
set_target_properties(Cube5-I PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Cube5-I PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Cube5-I ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})


add_executable(Minicube-II-1 Minicube-II-1.cpp TestBase.cpp)
set_target_properties(Minicube-II-1 PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(Minicube-II-1 PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(Minicube-II-1 ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(snake snake.cpp TestBase.cpp)
set_target_properties(snake PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(snake PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(snake ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})

add_executable(test test.cpp TestBase.cpp)
set_target_properties(test PROPERTIES COMPILE_FLAGS "${OPENRAVE_CXXFLAGS}")
set_target_properties(test PROPERTIES LINK_FLAGS "${OPENRAVE_LINK_FLAGS}")
target_link_libraries(test ${OPENRAVE_LIBRARY} ${OPENRAVE_CORE_LIBRARY})



