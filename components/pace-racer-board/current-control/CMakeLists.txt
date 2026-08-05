cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
  "../../../components/"
)

set(
  COMPONENTS
  "main esptool_py pace-racer-board"
  CACHE STRING
  "List of components to include"
  )

project(pace_racer_current_control)

set(CMAKE_CXX_STANDARD 20)
