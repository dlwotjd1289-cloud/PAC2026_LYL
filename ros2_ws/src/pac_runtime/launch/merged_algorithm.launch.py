"""Start the integration profile with Taehyeon's layer placement by default.

    ros2 launch pac_runtime merged_algorithm.launch.py repo:=/path/to/repo \
      order_file:=/path/to/order.json

No workcell, motion executor or simulator is started by this launch file.
The experimental merged ranker remains selectable with a matching custom
lookahead_config whose placer is merged. Jaesung evaluation is separate.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    config_dir = LaunchConfiguration("config_dir")
    cfg = lambda name: PathJoinSubstitution([config_dir, name])  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("repo", description="PAC2026 algorithm repository checkout"),
        DeclareLaunchArgument("order_file", description="order JSON (pac_runtime/order.py)"),
        DeclareLaunchArgument("config_dir", default_value=PathJoinSubstitution(
            [repo, "config", "jaesung_taehyeon"])),
        DeclareLaunchArgument("merged_config", default_value=cfg("merged.yaml")),
        DeclareLaunchArgument("ranker", default_value="layer",
            description="placement ranker; merged is experimental and requires a matching lookahead_config"),
        DeclareLaunchArgument("lookahead_config", default_value=cfg("lookahead.yaml")),
        DeclareLaunchArgument("robot_config", default_value=PathJoinSubstitution(
            [repo, "config", "taehyeon", "robot_check.yaml"]),
            description="existing stage-6 robot feasibility parameters"),
        Node(
            package="pac_runtime",
            executable="runtime_node",
            output="screen",
            parameters=[{
                "order_file": LaunchConfiguration("order_file"),
                "candidates_config": cfg("candidates.yaml"),
                "highlevel_config": cfg("highlevel.yaml"),
                "lookahead_config": LaunchConfiguration("lookahead_config"),
                "runtime_config": cfg("runtime.yaml"),
                "robot_config": LaunchConfiguration("robot_config"),
                "ranker": LaunchConfiguration("ranker"),
                "merged_config": LaunchConfiguration("merged_config"),
            }],
        ),
    ])
