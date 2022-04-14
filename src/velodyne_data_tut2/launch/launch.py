from launch import LaunchDescription
from launch_ros.actions import Node
import launch
from ament_index_python import get_package_share_directory
import os

point_cloud_node_prefix = get_package_share_directory('velodyne_data_tut2')
point_cloud_node_prefix_param_file = os.path.join(point_cloud_node_prefix,
                                            'param/params.yaml')
def generate_launch_description():
    return LaunchDescription([
       Node(
            package='velodyne_data_tut2',
            executable='point_cloud',
            # Namespadce e bakarak aynı ıkı npodu farklı ısımlerle calıstırabılırsın
            namespace='deneme',
            parameters=[point_cloud_node_prefix_param_file],


        )
    ])
# print("hello")
# # print(point_cloud_node_prefix_param_file)
# print(point_cloud_node_prefix)




# def generate_launch_description():
#     point_cloud_node = Node(
#         package='velodyne_data_tut2',
#         executable='point_cloud',
#         namespace='deneme',
#         parameters=[point_cloud_node],
#         prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=yes'],
#         output='screen'
    #
    # )
    # return launch.LaunchDescription([point_cloud_node])