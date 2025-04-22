from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start SLAM Toolbox online mode
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_link",
                "scan_topic": "/scan", #jackal1/sensors/lidar3d_0/scan",
                "mode": "mapping",
                "resolution": 0.05,  # Grid size (adjust as needed)
                "map_update_interval": 2.0,
                "use_scan_matching": True,
                "use_loop_closing": False,
                "queue_size": 10
            }]
        ),

        # Start Rviz for visualization
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=["-d", "/path/to/your/rviz_config.rviz"]
        )
    ])