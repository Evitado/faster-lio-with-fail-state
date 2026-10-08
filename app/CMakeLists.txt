add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME}
        )
if (FASTER_LIO_TRACY AND FASTER_LIO_TRACY_MEMORY)
    target_compile_definitions(run_mapping_online PRIVATE FASTER_LIO_TRACY_MEMORY)
endif ()
install(TARGETS run_mapping_online
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})