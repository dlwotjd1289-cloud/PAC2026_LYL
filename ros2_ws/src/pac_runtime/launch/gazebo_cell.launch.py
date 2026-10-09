"""Runtime node + Gazebo cell driver for the pac2026-ahead HDR50-22 workcell.

Start the Gazebo workcell first (pac2026-ahead workspace, other terminal),
with the robot on the 0.5 m pedestal:
    ros2 launch ~/pac-mission1-shared/tools/runtime/launch/hdr50_pedestal_workcell.launch.py
then (this workspace):
    ros2 launch pac_runtime gazebo_cell.launch.py \
      repo:=$HOME/pac-mission1-shared ranker:=donghan

Both nodes use config/taehyeon/robot_check_gazebo.yaml (robot base at world
(1.35, 0.15, 0.5), pallet centre (1.35, -1.0)), so the joint values match the
robot in Gazebo. Stage 4 always decides with the look-ahead search over the
boxes the cell driver publishes on /pac/conveyor_preview; ``ranker:=layer``
(flat layers / side contact) is the default low-level order. The
Donghan ranker uses its deterministic EMS-backed heuristic when
``ranker_model_path`` is empty. Learned models require their matching
``ranker_config`` to preserve the rollout horizon/scenario contract.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    repo = LaunchConfiguration("repo")
    cfg = lambda name: PathJoinSubstitution([repo, "config", "taehyeon", name])  # noqa: E731
    order = LaunchConfiguration("order_file")
    return LaunchDescription([
        DeclareLaunchArgument("repo", description="pac-mission1-shared checkout (for config/taehyeon)"),
        DeclareLaunchArgument("order_file", default_value=PathJoinSubstitution(
            [LaunchConfiguration("repo"), "config", "taehyeon", "example_order.json"])),
        DeclareLaunchArgument("speed_scale", default_value="0.3"),
        DeclareLaunchArgument("seed", default_value="0"),
        DeclareLaunchArgument("visible_boxes", default_value="5",
                              description="boxes after the current one the conveyor camera sees"),
        DeclareLaunchArgument("ranker", default_value="layer",
                              description="low-level ordering: layer, dblf or donghan"),
        DeclareLaunchArgument("ranker_model_path", default_value="",
                              description="optional validated donghan model; empty uses deterministic heuristic"),
        DeclareLaunchArgument("ranker_config", default_value="",
                              description="matching Donghan planner config (horizon/scenario contract)"),
        DeclareLaunchArgument("ranker_seed", default_value="7"),
        Node(package="pac_runtime", executable="runtime_node", output="screen", parameters=[{
            "order_file": order,
            "candidates_config": cfg("candidates.yaml"),
            "highlevel_config": cfg("highlevel.yaml"),
            "lookahead_config": cfg("lookahead.yaml"),
            "runtime_config": cfg("runtime.yaml"),
            "robot_config": cfg("robot_check_gazebo.yaml"),
            "ranker": LaunchConfiguration("ranker"),
            "ranker_model_path": LaunchConfiguration("ranker_model_path"),
            "ranker_config": LaunchConfiguration("ranker_config"),
            "ranker_seed": LaunchConfiguration("ranker_seed"),
        }]),
        Node(package="pac_runtime", executable="gazebo_cell", output="screen", parameters=[{
            "order_file": order,
            "robot_config": cfg("robot_check_gazebo.yaml"),
            "speed_scale": LaunchConfiguration("speed_scale"),
            "seed": LaunchConfiguration("seed"),
            "visible_boxes": LaunchConfiguration("visible_boxes"),
        }]),
    ])
