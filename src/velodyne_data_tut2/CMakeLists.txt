cmake_minimum_required(VERSION 3.8)
project(velodyne_data_tut2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()


find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(point_cloud_node SHARED
        include/velodyne_data_tut2/UdacityTut.h
        include/velodyne_data_tut2/RosRelated.h
        include/velodyne_data_tut2/PclStuff.h
        src/UdacityTut.cpp
        src/RosRelated.cpp
        src/PclStuff.cpp
        )

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)


target_link_libraries(point_cloud_node
        ${PCL_LIBRARIES}
        )


# works as main  plugın takes string be carefull about write string correctly
rclcpp_components_register_node(point_cloud_node
        PLUGIN "UdacityTutNO::UdacityTut"
        EXECUTABLE point_cloud
        )

# Ament Exporting
ament_auto_package(
        INSTALL_TO_SHARE
        launch
)
