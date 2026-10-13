"""ros2 launch bvi_planner bvi.launch.py difficulty:=hard [seed:=42] [rviz:=true]
                  [rviz_config:=bvi_policy.rviz] [rviz_prefix:=...]"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("bvi_planner"))
    params = str(share / "config" / "params.yaml")
    difficulty = LaunchConfiguration("difficulty")
    seed = LaunchConfiguration("seed")

    return LaunchDescription([
        DeclareLaunchArgument("difficulty", default_value="medium",
                              description="easy | medium | hard"),
        DeclareLaunchArgument("seed", default_value="-1",
                              description="explicit map seed (>= 0 overrides difficulty)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value="bvi.rviz",
                              description="bvi.rviz | bvi_policy.rviz (policy arrows on)"),
        DeclareLaunchArgument("rviz_prefix", default_value="",
                              description="e.g. '/opt/VirtualGL/bin/vglrun -d egl' on a VNC display"),
        # Identity world->map transform so RViz has a valid "map" TF frame.
        Node(package="tf2_ros", executable="static_transform_publisher", name="map_frame",
             arguments=["--frame-id", "world", "--child-frame-id", "map"], output="log"),
        Node(package="bvi_planner", executable="planner_node", name="bvi_planner",
             output="screen",
             parameters=[params, {"difficulty": difficulty, "seed": seed}]),
        Node(package="bvi_planner", executable="robot_animation_node", name="bvi_robot",
             output="screen", parameters=[params]),
        Node(package="bvi_planner", executable="robot_animation_node", name="abvi_robot",
             output="screen", parameters=[params]),
        Node(package="bvi_planner", executable="optimality_checker_node",
             name="optimality_checker", output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
             arguments=["-d", PathJoinSubstitution([str(share / "rviz"),
                                                    LaunchConfiguration("rviz_config")])],
             condition=IfCondition(LaunchConfiguration("rviz")),
             prefix=LaunchConfiguration("rviz_prefix")),
    ])
