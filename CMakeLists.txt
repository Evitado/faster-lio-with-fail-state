cmake_minimum_required(VERSION 3.5)
project(faster_lio)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 23)

include(cmake/packages.cmake)

# profiling (see profiling/README.md): compiled in, switched on at runtime with ~profiling_enable
option(FASTER_LIO_TRACY "Compile in the Tracy profiler client (dormant unless ~profiling_enable)" ON)
set(FASTER_LIO_TRACY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/foreign/tracy"
    CACHE PATH "Tracy source tree providing public/TracyClient.cpp (same commit as tracy-capture)")
set(FASTER_LIO_TRACY_CAPTURE "${CMAKE_CURRENT_SOURCE_DIR}/profiling/tracy-tools/bin/tracy-capture"
    CACHE FILEPATH "tracy-capture the node starts for ~profiling_enable (falls back to tracy-capture in PATH)")

add_subdirectory(src)
add_subdirectory(app)

if (CATKIN_ENABLE_TESTING)
    catkin_add_gtest(${PROJECT_NAME}-test-point-cloud test/point_cloud_test.cc)
    if (TARGET ${PROJECT_NAME}-test-point-cloud)
        # header-only code under test: no need for the library (TBB, Tracy)
        target_include_directories(${PROJECT_NAME}-test-point-cloud PRIVATE
                ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME}-test-point-cloud ${catkin_LIBRARIES})
    endif ()
endif ()

foreach(dir config launch)
  install(DIRECTORY ${dir}/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

