cmake_minimum_required(VERSION 2.8.12)
project(block_sorting_aikido)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# ===============================================================================
# Dependencies
#

find_package(DART REQUIRED)
include_directories(${DART_INCLUDE_DIRS})

find_package(aikido REQUIRED COMPONENTS
  constraint
  distance
  perception
  planner_ompl
  planner_parabolic
  rviz
  statespace
  util)
include_directories(${aikido_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

find_package(libherb REQUIRED)
include_directories(${libherb_INCLUDE_DIRS})

find_package(pr_ordata REQUIRED)
include_directories(${pr_ordata_INCLUDE_DIRS})

# ===============================================================================
# Executables
#

add_executable(block_sorting
    src/block_sorting.cpp
)

target_link_libraries(block_sorting
  ${DART_LIBRARIES}
  ${aikido_LIBRARIES}
  libherb)

# ===============================================================================
# Installation
#

install(TARGETS block_sorting RUNTIME DESTINATION bin)

#================================================================================
# Formatting
#

find_program(
  CLANG_FORMAT_EXECUTABLE
  NAMES clang-format clang-format-3.9 clang-format-3.8
)

if(CLANG_FORMAT_EXECUTABLE)
  message(STATUS "Found clang-format.")

  file(GLOB_RECURSE ALL_SOURCE_FILES
    LIST_DIRECTORIES false
    include/*.h include/*.hpp src/*.c src/*.cpp)
  list(LENGTH ALL_SOURCE_FILES NUM_SOURCE_FILES)

  add_custom_target(format
    COMMAND ${CMAKE_COMMAND} -E echo "Formatting ${NUM_SOURCE_FILES} files..."
    COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${ALL_SOURCE_FILES}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
else()

  message(STATUS "Looking for clang-format - NOT found, please install clang-format to enable automatic code formatting.")

endif()

