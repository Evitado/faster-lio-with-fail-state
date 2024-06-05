add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )
install(TARGETS run_mapping_online
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})