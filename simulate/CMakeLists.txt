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
find_package(legged_sdk REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

# MuJoCo's libmujoco.so statically links tinyxml2 and exports its symbols. Those
# shadow the system libtinyxml2 that FastDDS (pulled in via legged_sdk) uses for
# XML parsing, so creating a DDS participant crashes. Force the system tinyxml2
# to be a direct dependency loaded before MuJoCo, keeping FastDDS on the ABI it
# was built against.
find_library(TINYXML2_LIBRARY NAMES tinyxml2)
if(NOT TINYXML2_LIBRARY)
  # Ubuntu ships no libtinyxml2.so dev symlink, only the versioned SONAME.
  find_file(TINYXML2_LIBRARY NAMES libtinyxml2.so.10
    PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
          /lib/${CMAKE_LIBRARY_ARCHITECTURE}
          /usr/local/lib)
endif()
if(NOT TINYXML2_LIBRARY)
  message(FATAL_ERROR "libtinyxml2 (FastDDS runtime dependency) not found")
endif()

include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/src
  /opt/legged_sdk/include
  )

file(GLOB SIM_SRC
    src/joystick/joystick.cc
)

set(SIM_DEPENDENCIES
    pthread
    -Wl,--no-as-needed
    ${TINYXML2_LIBRARY}
    -Wl,--as-needed
    mujoco::libmujoco_simulate
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
  ${SIM_DEPENDENCIES} )

add_executable(test test/test_legged_sdk.cpp)
target_link_libraries(test legged_sdk)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)
