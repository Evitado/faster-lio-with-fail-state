cmake_minimum_required(VERSION 3.5)
project(faster_lio)

set(DEFAULT_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS}")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

include(cmake/packages.cmake)

#definitions
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# options
option(WITH_IVOX_NODE_TYPE_PHC "Use PHC instead of default ivox node" OFF)

if (WITH_IVOX_NODE_TYPE_PHC)
    message("USING_IVOX_NODE_TYPE_PHC")
    add_definitions(-DIVOX_NODE_TYPE_PHC)
else ()
    message("USING_IVOX_NODE_TYPE_DEFAULT")
endif()

# profiling (see profiling/README.md): compiled in, switched on at runtime with ~profiling_enable
option(FASTER_LIO_TRACY "Compile in the Tracy profiler client (dormant unless ~profiling_enable)" ON)
set(FASTER_LIO_TRACY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/foreign/tracy"
    CACHE PATH "Tracy source tree providing public/TracyClient.cpp (same commit as tracy-capture)")
set(FASTER_LIO_TRACY_CAPTURE "${CMAKE_CURRENT_SOURCE_DIR}/profiling/tracy-tools/bin/tracy-capture"
    CACHE FILEPATH "tracy-capture the node starts for ~profiling_enable (falls back to tracy-capture in PATH)")

add_subdirectory(src)
add_subdirectory(app)

foreach(dir config launch)
  install(DIRECTORY ${dir}/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

