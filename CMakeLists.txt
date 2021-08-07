#	State Key Lab of CAD&CG Zhejiang Unv.
#
#	Author: 
#          Yicun Zheng (3130104113@zju.edu.cn)
#          Haoran Sun (hrsun@zju.edu.cn)
#          Jin Huang (hj@cad.zju.edu.cn)
#
#	Copyright (c) 2004-2021 <Jin Huang>
#	All rights reserved.
#
#	Licensed under the MIT License.

cmake_minimum_required(VERSION 3.14 FATAL_ERROR)


project(
  hausdorff
  VERSION 0.1.0
  LANGUAGES CXX
)

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

include(cmake/CPM.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


CPMAddPackage(
  NAME zjucad.matrix
  GITHUB_REPOSITORY ZJUCADGeoSim/matrix
  GIT_TAG v0.1.0
)

CPMAddPackage(
  NAME cxxopts
  GITHUB_REPOSITORY jarro2783/cxxopts
  GIT_TAG v2.2.1
  OPTIONS "CXXOPTS_BUILD_EXAMPLES OFF" "CXXOPTS_BUILD_TESTS OFF"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

add_subdirectory(./src/core)
add_subdirectory(./src/mesh)
add_subdirectory(./src/bvh)
add_subdirectory(./src/hausdorff)
add_subdirectory(./src/io)
add_subdirectory(./src/app)
