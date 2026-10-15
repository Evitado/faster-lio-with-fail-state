find_package(TBB REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        std_srvs
        roscpp
        tf
        message_generation
        )

add_message_files(
        FILES
        Pose6D.msg
)

add_service_files(
        FILES
        SaveTrajectory.srv
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs std_srvs message_runtime
        DEPENDS EIGEN3
)
