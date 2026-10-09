"""Start the AHEAD runtime node (stages 1-8 decisions) with taehyeon's configs.

    ros2 launch pac_runtime runtime.launch.py repo:=/path/to/pac-mission1-shared \
      order_file:=/path/order.json ranker:=donghan

Stage 4 always decides with the look-ahead search (config/taehyeon/lookahead.yaml)
over the boxes the plant publishes on /pac/conveyor_preview.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    cfg = lambda name: PathJoinSubstitution([repo, "config", "taehyeon", name])  # noqa: E731
    return LaunchDescription([
        DeclareLaunchArgument("repo", description="pac-mission1-shared checkout (for config/taehyeon)"),
        DeclareLaunchArgument("order_file", description="order list JSON (see pac_runtime/order.py)"),
        DeclareLaunchArgument("ranker", default_value="layer",
                              description="low-level ordering: layer, dblf or donghan"),
        DeclareLaunchArgument("ranker_model_path", default_value="",
                              description="optional validated donghan model; empty uses deterministic heuristic"),
        DeclareLaunchArgument("ranker_config", default_value="",
                              description="matching Donghan planner config (horizon/scenario contract)"),
        DeclareLaunchArgument("ranker_seed", default_value="7"),
        Node(
            package="pac_runtime",
            executable="runtime_node",
            output="screen",
            parameters=[{
                "order_file": LaunchConfiguration("order_file"),
                "candidates_config": cfg("candidates.yaml"),
                "highlevel_config": cfg("highlevel.yaml"),
                "lookahead_config": cfg("lookahead.yaml"),
                "runtime_config": cfg("runtime.yaml"),
                "robot_config": cfg("robot_check.yaml"),
                "ranker": LaunchConfiguration("ranker"),
                "ranker_model_path": LaunchConfiguration("ranker_model_path"),
                "ranker_config": LaunchConfiguration("ranker_config"),
                "ranker_seed": LaunchConfiguration("ranker_seed"),
            }],
        ),
    ])
