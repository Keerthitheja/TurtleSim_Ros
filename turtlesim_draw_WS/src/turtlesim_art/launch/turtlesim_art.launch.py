from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    image_path_argument = DeclareLaunchArgument(
        "image_path",
        default_value=None,
        description="Path to image file to"
    )

    return LaunchDescription([
        image_path_argument,
        Node(
        package = "turtlesim_art",
        executable = "turtle_sim_draw",
        name = "turtle_sim_draw",
        parameters=[{'image_path': LaunchConfiguration('image_path')}]
        ),
        
        Node(
        package = "turtlesim",
        namespace = "turtlesim1",
        executable = "turtlesim_node",
        name = "sim"
        ),

    ])
