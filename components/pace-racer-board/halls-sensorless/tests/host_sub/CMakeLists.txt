cmake_minimum_required(VERSION 3.20)
project(pr_telem_sub CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# managed_components lives two levels up; populate it with `idf.py build`
# in halls-sensorless/ before configuring this project.
set(MC "${CMAKE_CURRENT_SOURCE_DIR}/../../managed_components")
if(NOT EXISTS "${MC}/espp__rtps")
  message(FATAL_ERROR "managed_components/espp__rtps not found -- run `idf.py build` in halls-sensorless/ first")
endif()

file(GLOB RTPS_SRC ${MC}/espp__rtps/src/*.cpp
                   ${MC}/espp__rtps/src/*/*.cpp)
file(GLOB DEP_SRC  ${MC}/espp__socket/src/*.cpp
                   ${MC}/espp__task/src/*.cpp
                   ${MC}/espp__thread_pool/src/*.cpp
                   ${MC}/espp__logger/src/*.cpp)

add_executable(telem_sub telem_sub.cpp ${RTPS_SRC} ${DEP_SRC})

target_include_directories(telem_sub PRIVATE
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
)
target_compile_definitions(telem_sub PRIVATE FMT_HEADER_ONLY=1)
