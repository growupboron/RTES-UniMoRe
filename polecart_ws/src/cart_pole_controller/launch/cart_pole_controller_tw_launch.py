from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('cart_pole_controller'),
        'config',
        'cart_pole_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='cart_pole_controller',
            executable='cart_pole_controller_tw',
            name='cart_pole_controller_tw',
            parameters=[param_file]
        )
    ])