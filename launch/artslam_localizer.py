'''

'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    localizer_config = os.path.join(
        get_package_share_directory('localizer_wrapper'),
        'config',
        'params.yaml'
    )

    artslam_localizer_node = LaunchDescription([
        Node(
            package='localizer_wrapper',
            executable='artslam_localizer',
            name='artslam_localizer',
            #prefix=['xterm -e gdb -ex run --args'],
            output='screen',
            parameters=[localizer_config]
        )
    ])


    return LaunchDescription([
        artslam_localizer_node,
    ])
