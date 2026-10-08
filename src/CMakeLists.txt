add_library(${PROJECT_NAME}
        laser_mapping.cc
        pointcloud_preprocess.cc
        options.cc
        )

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

target_include_directories(${PROJECT_NAME} PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        TBB::tbb
        )

if (FASTER_LIO_TRACY)
    if (NOT EXISTS "${FASTER_LIO_TRACY_DIR}/public/TracyClient.cpp")
        message(FATAL_ERROR "FASTER_LIO_TRACY: no Tracy client in FASTER_LIO_TRACY_DIR=${FASTER_LIO_TRACY_DIR}")
    endif ()
    message(STATUS "faster_lio: Tracy client compiled in (${FASTER_LIO_TRACY_DIR}), recorder ${FASTER_LIO_TRACY_CAPTURE}")
    # one Tracy client per process, compiled into the library the executables link
    target_sources(${PROJECT_NAME} PRIVATE "${FASTER_LIO_TRACY_DIR}/public/TracyClient.cpp" profiling.cc)
    target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${FASTER_LIO_TRACY_DIR}/public")
    # a client that does not exist until profiling::Start (manual lifetime), records only while the recorder is
    # connected (on demand), listens on localhost only, and leaves out what needs kernel tracing permissions.
    # Not TRACY_NO_CALLSTACK: without the symbol thread it brings, source file queries from the recorder (it sends
    # them for nix store binaries) are never answered and the recorder never finishes.
    target_compile_definitions(${PROJECT_NAME} PUBLIC
            TRACY_ENABLE FASTER_LIO_TRACY
            TRACY_DELAYED_INIT TRACY_MANUAL_LIFETIME TRACY_ON_DEMAND TRACY_ONLY_LOCALHOST TRACY_NO_BROADCAST
            TRACY_NO_SYSTEM_TRACING TRACY_NO_SAMPLING TRACY_NO_CODE_TRANSFER TRACY_NO_FRAME_IMAGE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE FASTER_LIO_TRACY_CAPTURE="${FASTER_LIO_TRACY_CAPTURE}")
    target_link_libraries(${PROJECT_NAME} dl)
endif ()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
