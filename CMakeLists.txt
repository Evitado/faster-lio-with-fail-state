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
set(FASTER_LIO_TRACY_SUBMODULE "${CMAKE_CURRENT_SOURCE_DIR}/foreign/tracy")
if (FASTER_LIO_TRACY)
    if (NOT EXISTS "${FASTER_LIO_TRACY_DIR}/public/TracyClient.cpp")
        # e.g. a clone without --recursive: build without profiling rather than not at all
        message(WARNING "faster_lio: no Tracy client in FASTER_LIO_TRACY_DIR=${FASTER_LIO_TRACY_DIR}, building without"
                " profiling (git submodule update --init foreign/tracy to have it)")
        set(FASTER_LIO_TRACY OFF)
    elseif (NOT FASTER_LIO_TRACY_DIR STREQUAL FASTER_LIO_TRACY_SUBMODULE
            AND EXISTS "${FASTER_LIO_TRACY_SUBMODULE}/public/TracyClient.cpp")
        # the dir is a cache variable: a build directory configured before the submodule keeps its old path
        message(WARNING "faster_lio: Tracy client from ${FASTER_LIO_TRACY_DIR}, not the foreign/tracy submodule. The"
                " recorder ${FASTER_LIO_TRACY_CAPTURE} must be built from the same Tracy commit, or recording fails;"
                " -DFASTER_LIO_TRACY_DIR=${FASTER_LIO_TRACY_SUBMODULE} to use the submodule")
    endif ()
endif ()

add_subdirectory(src)
add_subdirectory(app)

if (CATKIN_ENABLE_TESTING)
    # the tests build the code under test from headers / sources instead of linking the library: no TBB, and
    # without FASTER_LIO_TRACY the profiling macros are no-ops
    function(faster_lio_add_test name)  # further arguments: sources
        catkin_add_gtest(${PROJECT_NAME}-test-${name} ${ARGN})
        if (TARGET ${PROJECT_NAME}-test-${name})
            target_include_directories(${PROJECT_NAME}-test-${name} PRIVATE
                    ${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
            target_link_libraries(${PROJECT_NAME}-test-${name} ${catkin_LIBRARIES})
            add_dependencies(${PROJECT_NAME}-test-${name} ${PROJECT_NAME}_gencpp)  # faster_lio/Pose6D.h
        endif ()
    endfunction()

    faster_lio_add_test(point-cloud test/point_cloud_test.cc)
    faster_lio_add_test(voxel-table test/voxel_table_test.cc)
    faster_lio_add_test(ivox test/ivox_test.cc)
    faster_lio_add_test(common-lib test/common_lib_test.cc)
    faster_lio_add_test(pointcloud-preprocess test/pointcloud_preprocess_test.cc src/pointcloud_preprocess.cc)
    faster_lio_add_test(imu-processing test/imu_processing_test.cc)
endif ()

foreach(dir config launch)
  install(DIRECTORY ${dir}/
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

