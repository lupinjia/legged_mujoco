cmake_minimum_required(VERSION 3.16)
project(legged_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/legged_sdk/lib/cmake")
find_package(mujoco REQUIRED)
find_package(OpenCV REQUIRED)
find_package(legged_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/src
  /opt/legged_sdk/include
  )

file(GLOB SIM_SRC
    src/joystick/joystick.cc
    src/mujoco/*.cc
)

set(SIM_DEPENDENCIES
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    legged_sdk
    boost_program_options
    fmt
)

add_executable(legged_mujoco 
  ${SIM_SRC} 
  src/main.cc)
target_link_libraries(legged_mujoco 
  ${SIM_DEPENDENCIES} 
  ${OpenCV_LIBS} )

add_executable(test test/test_legged_sdk.cpp)
target_link_libraries(test legged_sdk)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)