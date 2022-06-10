# Copyright Moses Turner, 2021.
# SPDX-License-Identifier: BSL-1.0
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address")

set(CMAKE_VERBOSE_MAKEFILE ON)

project(sk-notes VERSION "0.0.1" LANGUAGES CXX C)

find_package(StereoKitC REQUIRED)
find_package(Eigen3 3 REQUIRED)
# find_package(realsense2 REQUIRED)


message("Found StereoKitC " ${StereoKitC_Found})

add_executable(
  sk-notes
  src/main.cpp
  src/demo_picker.h
  src/demo_picker.cpp
  src/demo_sprites.h
  src/demo_ui.h
  src/demo_sprites.cpp
  src/demo_basics.h
  src/scene.h
  src/demo_lines.h
  src/scene.cpp
  src/demo_ui.cpp
  src/demo_basics.cpp
  src/demo_lines.cpp
)

target_link_libraries(
  sk-notes
  -L$oxrinstall/lib
  StereoKitC
)
