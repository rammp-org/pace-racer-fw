cmake_minimum_required(VERSION 3.20)
project(atos_drive_host_supervisor CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# The supervisor and the FOC math helpers, natively, once per motor profile.
# Header-only on both sides: no espp, no IDF, esp_timer is a stub clock.
#   cmake -S . -B build && cmake --build build && ctest --test-dir build
set(MAIN "${CMAKE_CURRENT_SOURCE_DIR}/../../main")
set(RAMMP "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../external/rammp-rtps/components")
enable_testing()
foreach(profile ninebot flatbot hub20)
  add_executable(test_supervisor_${profile} test_supervisor.cpp)
  target_include_directories(test_supervisor_${profile} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/stub ${MAIN}
    ${RAMMP}/rammp_rtps_interface/include ${RAMMP}/rammp_rtps_messages/include
    ${RAMMP}/rammp_rtps_actions/include ${RAMMP}/rammp_rtps_services/include)
  if(profile STREQUAL "hub20")
    target_compile_definitions(test_supervisor_${profile} PRIVATE ATOS_MOTOR_HUB20)
  elseif(profile STREQUAL "flatbot")
    target_compile_definitions(test_supervisor_${profile} PRIVATE ATOS_MOTOR_FLATBOT)
  endif()
  add_test(NAME supervisor_${profile} COMMAND test_supervisor_${profile})
endforeach()
