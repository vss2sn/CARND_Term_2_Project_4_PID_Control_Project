project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)

set(uWS_CUSTOM_INSTALL_DIR "NO")  # NO if system-wide install

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

# NOTE:
# Minor addition for convenience, I have not installed
# the uWebSockets library system wide.
if(${uWS_CUSTOM_INSTALL_DIR} MATCHES "YES")
include_directories(../uWebSockets/bin/include)
link_directories(../uWebSockets/bin/lib64)
endif(${uWS_CUSTOM_INSTALL_DIR} MATCHES "YES")

add_executable(pid ${sources})

target_link_libraries(pid z ssl uv uWS)
