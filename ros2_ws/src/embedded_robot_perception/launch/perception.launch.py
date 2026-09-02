from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    robot_id = LaunchConfiguration("robot_id")
    ros_namespace = LaunchConfiguration("ros_namespace")
    backend = LaunchConfiguration("backend")
    require_gpu = LaunchConfiguration("require_gpu")
    component = ComposableNode(
        package="embedded_robot_perception",
        plugin="embedded_robot_perception::PerceptionNode",
        name="perception",
        namespace="",
        parameters=[
            {
                "robot_id": robot_id,
                "backend": backend,
                "require_gpu": require_gpu,
                "mock_reachability": False,
            }
        ],
        extra_arguments=[{"use_intra_process_comms": True}],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_id", default_value="mock-perception-01"),
            DeclareLaunchArgument("ros_namespace", default_value="mock_perception_01"),
            DeclareLaunchArgument("backend", default_value="auto"),
            DeclareLaunchArgument("require_gpu", default_value="false"),
            ComposableNodeContainer(
                name="perception_container",
                namespace=ros_namespace,
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[component],
                output="screen",
            ),
        ]
    )
