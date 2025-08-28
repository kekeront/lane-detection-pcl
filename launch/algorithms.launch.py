from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ransac_seg",
            executable="astar",
            name="astar_node",
            output="screen"
        ),
        Node(
            package="ransac_seg",
            executable="dfs",
            name="dfs_node",
            output="screen"
        ),
        Node(
            package="ransac_seg",
            executable="curve",
            name="curve_node",
            output="screen"
        ),
        Node(
            package="ransac_seg",
            executable="djikstra",
            name="djikstra_node",
            output="screen"
        ),
        Node(
            package="ransac_seg",
            executable="ransac_node",
            name="ransac_node",
            output="screen"
        ),
    ])
