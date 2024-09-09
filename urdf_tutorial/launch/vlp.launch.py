import os
import ament_index_python.packages
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "urdf_tutorial"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        # launch_arguments={"verbose": "true", "world": "empty.world"}.items(),
        launch_arguments={"verbose": "true", "world": os.path.join(get_package_share_directory(package_name), "world", "pyramid.world")}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot","-x", "0", "-y", "0", "-z", "1.1"],
        output="screen",
    )

    rviz_config = os.path.join(
        ament_index_python.packages.get_package_share_directory('pcd_filter'),
        'rviz', 'rviz.rviz'
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        output='both',
        arguments=['-d', rviz_config]
    )

    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            rviz,
        ]
    )
