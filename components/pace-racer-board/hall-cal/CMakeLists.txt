cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(COMPONENTS
  "main esptool_py"
  CACHE STRING
  "List of components to include"
)

project(pace_racer_hall_cal)

set(CMAKE_CXX_STANDARD 20)
