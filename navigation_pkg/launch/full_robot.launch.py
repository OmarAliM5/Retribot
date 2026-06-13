from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():

    reset_old_nodes = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'pkill -f position_pid_node || true; '
            'pkill -f mecanum_controller_node || true; '
            'pkill -f mecanum_odometry_node || true; '
            'pkill -f path_follower_node || true; '
            'pkill -f imu_node || true; '
            'sleep 1'
        ],
        output='screen'
    )

    start_nodes = TimerAction(
        period=1.5,
        actions=[

            # ExecuteProcess(
            #     cmd=[
            #         'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            #         'serial', '--dev', '/dev/ttyACM0', '-b', '921600'
            #     ],
            #     output='screen'
            # ),

            Node(
                package='imu_pkg',
                executable='imu_node',
                output='screen'
            ),
            
            Node(
                package='imu_pkg',
                executable='position_pid_node',
                output='screen'
            ),

            Node(
                package='mecanum_controller_pkg',
                executable='mecanum_controller_node',
                output='screen'
            ),

            Node(
                package='mecanum_controller_pkg',
                executable='mecanum_odometry_node',
                output='screen'
            ),
            Node(
                package='navigation_pkg',
                executable='path_follower_node',
                output='screen'
            ),

            #TimerAction(
            #    period=5.0,
            #    actions=[
            #       Node(
            #           package='navigation_pkg',
            #          executable='path_follower_node',
            #           output='screen'
            #      )
            #  ]
            #),
        ]
    )

    return LaunchDescription([
        reset_old_nodes,
        start_nodes
    ])