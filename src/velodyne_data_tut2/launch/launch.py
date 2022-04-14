from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velodyne_data_tut2',
            executable='point_cloud',
            # Namespadce e bakarak aynı ıkı npodu farklı ısımlerle calıstırabılırsın
            # namespace='turtlesim2',
            # Burdan point_cloud/sim olarak topic ismi verilebilir
            # name='sim'
        )

        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])
