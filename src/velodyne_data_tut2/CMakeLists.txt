cmake_minimum_required(VERSION 3.8)
project(velodyne_data_tut2)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

#set(PCL_DIR "/home/zeys/libraries/libpcl-1.11.1/share/pcl-1.11/")
find_package(ament_cmake_auto REQUIRED)
#find_package(PCL 1.11.1 REQUIRED)
find_package(PCL REQUIRED)

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_node
        src/UdacityTut.cpp
        src/RosRelated.cpp
        src/PclStuff.cpp
#        src/publisher_member_function.cpp
        )

target_link_libraries(point_cloud_node
        ${PCL_LIBRARIES}
        )

#target_include_directories(point_cloud_node
#        PUBLIC
#        $<INSTALL_INTERFACE:include>
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        PRIVATE
#        ${CMAKE_CURRENT_SOURCE_DIR}/src
#        )

ament_target_dependencies(point_cloud_node
        rclcpp
        std_msgs
        sensor_msgs
        )

#target_link_libraries(MyLib PRIVATE ssl crypto)

#target_include_directories(point_cloud_node PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>
#        )



install(TARGETS point_cloud_node
        DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
        )




##add_definitions(point_cloud_node ${PCL_DEFINITIONS})
##
##
##include_directories(point_cloud_node ${PCL_INCLUDE_DIRS})
#
#target_link_libraries(point_cloud_node
#        ${PCL_LIBRARIES}
#        )
#
#
#target_include_directories(point_cloud_node
#        PUBLIC
#        $<INSTALL_INTERFACE:include>
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        PRIVATE
#        ${CMAKE_CURRENT_SOURCE_DIR}/src
#        )
#
#
#install(TARGETS point_cloud_node
#        DESTINATION lib/${PROJECT_NAME})
#
#
## install the launch directory
#install(DIRECTORY
#        #        launch
#        DESTINATION share/${PROJECT_NAME}/
#        )
#
#
#ament_package()
#
##rclcpp_components_register_node(talker_node
##        PLUGIN "MinimalPublisher"
##        EXECUTABLE velodyne_data_tut
##        )
#
###TODO ADDED FOR MY PROJECT
##rclcpp_components_register_node(talker_node
##        PLUGIN "UdacityTut"
##        EXECUTABLE velodyne_data_tut2
##        )
#
##ament_target_dependencies(talker rclcpp std_msgs)
#
#
###TODO :ADD CPP FİLES HERE
##install(TARGETS
##        talker
##        DESTINATION lib/${PROJECT_NAME})
##if(BUILD_TESTING)
##  find_package(ament_lint_auto REQUIRED)
##  # the following line skips the linter which checks for copyrights
##  # uncomment the line when a copyright and license is not present in all source files
##  #set(ament_cmake_copyright_FOUND TRUE)
##  # the following line skips cpplint (only works in a git repo)
##  # uncomment the line when this package is not in a git repo
##  #set(ament_cmake_cpplint_FOUND TRUE)
##  ament_lint_auto_find_test_dependencies()
##endif()
##
##ament_package()


ament_package()