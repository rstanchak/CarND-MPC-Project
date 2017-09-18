project(MPC)

cmake_minimum_required (VERSION 2.8)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O0)
set(CXX_FLAGS "-Wall -Werror")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")


set(sources src/MPC.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)

#include_directories(src/Eigen-3.3.4)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc ipopt z ssl uv uWS)

if(${MYBUILD})
    add_definitions(-DCONFIG_UWS13_COMPAT)
    add_definitions(-DCONFIG_MYBUILD)
    target_link_libraries(mpc ssl)
endif()
