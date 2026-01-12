import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tiago_pro_gazebo"),
                "launch",
                "tiago_pro_gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world_name": "my_world", 
            "use_sim_time": "True",
            "moveit": "False",
            "extra_gazebo_args": "--verbose"
        }.items()
    )
    
    table_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "table",
	    "-file", os.path.expanduser("~/exchange/ros2_ws/src/models/table/model.sdf"),
            "-x", "1.0", 
            "-y", "0.0", 
            "-z", "0.0"
        ],
        output="screen"
    )
    
    cube_spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "rubiks_cube",
            "-file", os.path.expanduser("~/exchange/ros2_ws/src/models/cube/cube.sdf"),
            "-x", "0.6285",
            "-y", "0.0255", 
            "-z", "0.43"
        ],
        output="screen"
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tiago_pro_moveit_config"),
                "launch",
                "move_group.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
            "kinematics_yaml": os.path.expanduser("~/exchange/ros2_ws/config/kinematics_tiago_fixed.yaml")
        }.items()
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("tiago_pro_moveit_config"),
                "launch",
                "moveit_rviz.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "True",
            "allow_trajectory_execution": "true"
        }.items()
    )

    move_to_target = Node(
        package="tiago_auto_move",
        executable="move_to_target",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    centroid_pub = Node(
        package="tiago_target_pose",
        executable="publish_centroid_pose",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        gazebo,
        table_spawn,
        TimerAction(period=5.0, actions=[cube_spawn]),
        TimerAction(period=10.0, actions=[moveit]),
        TimerAction(period=20.0, actions=[rviz]),
        TimerAction(period=35.0, actions=[move_to_target]),
        TimerAction(period=45.0, actions=[centroid_pub]),
    ])
