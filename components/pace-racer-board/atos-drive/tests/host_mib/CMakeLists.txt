cmake_minimum_required(VERSION 3.20)
project(pr_mib_stub CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# espp builds natively. managed_components lives two levels up; populate it with
# `idf.py build` in atos-drive/ first, then apply
# ../../../halls-sensorless/tests/host_sub/espp-multicast-interface.patch (see README).
set(MC "${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components")
set(RAMMP "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../external/rammp-rtps/components")
if(NOT EXISTS "${MC}/espp__rtps")
  message(FATAL_ERROR "managed_components/espp__rtps not found -- run `idf.py build` in atos-drive/ first")
endif()

file(GLOB RTPS_SRC ${MC}/espp__rtps/src/*.cpp ${MC}/espp__rtps/src/*/*.cpp)
file(GLOB DEP_SRC ${MC}/espp__socket/src/*.cpp ${MC}/espp__task/src/*.cpp
                  ${MC}/espp__thread_pool/src/*.cpp ${MC}/espp__logger/src/*.cpp
                  ${MC}/espp__timer/src/timer.cpp)

add_executable(mib_stub mib_stub.cpp ${RTPS_SRC} ${DEP_SRC})
target_include_directories(mib_stub PRIVATE
  ${RAMMP}/rammp_rtps_messages/include
  ${MC}/espp__rtps/include
  ${MC}/espp__cdr/include
  ${MC}/espp__cdr/detail/cdr/include
  ${MC}/espp__reflect_cpp/detail/reflect-cpp/include
  ${MC}/espp__format/include
  ${MC}/espp__format/detail/fmt/include
  ${MC}/espp__base_component/include
  ${MC}/espp__logger/include
  ${MC}/espp__task/include
  ${MC}/espp__thread_pool/include
  ${MC}/espp__socket/include
  ${MC}/espp__timer/include
)
target_compile_definitions(mib_stub PRIVATE FMT_HEADER_ONLY=1)
