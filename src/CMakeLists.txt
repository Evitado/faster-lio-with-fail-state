add_library(${PROJECT_NAME}
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        utils.cc
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        tbb
        glog::glog
        yaml-cpp
        )

target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

if (FASTER_LIO_TRACY)
    if (NOT EXISTS "${FASTER_LIO_TRACY_DIR}/public/TracyClient.cpp")
        message(FATAL_ERROR "FASTER_LIO_TRACY: no Tracy client in FASTER_LIO_TRACY_DIR=${FASTER_LIO_TRACY_DIR}")
    endif ()
    message(STATUS "faster_lio: Tracy profiling enabled (${FASTER_LIO_TRACY_DIR})")
    # one Tracy client per process, compiled into the library the executables link
    target_sources(${PROJECT_NAME} PRIVATE "${FASTER_LIO_TRACY_DIR}/public/TracyClient.cpp")
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${FASTER_LIO_TRACY_DIR}/public")
    target_compile_definitions(${PROJECT_NAME} PUBLIC TRACY_ENABLE FASTER_LIO_TRACY)
    target_link_libraries(${PROJECT_NAME} dl)
endif ()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
