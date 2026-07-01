import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('multi_uav_sim')
    
    # Path to world file
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'cramped_env.world'])
    
    # Path to gazebo_ros launch file
    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])
    
    # Include Gazebo launch with our world
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world_path}.items()
    )
    
    # Path to drone URDF
    urdf_path = os.path.join(
        os.path.expanduser('~'), # inside WSL it will be mounted
        '/mnt/c/Users/tecim/.gemini/antigravity-ide/scratch/multi-uav-path-planning/ros2_ws/src/multi_uav_sim/models/drone.urdf'
    )
    
    # Start positions
    starts = [
        [-6.5, -10.0, 1.5],
        [-3.8, -10.0, 2.4],
        [-1.6, -10.0, 2.2]
    ]
    
    # Nodes to spawn 3 drones
    spawn_nodes = []
    for i, pos in enumerate(starts):
        spawn_nodes.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', f'uav_{i}',
                    '-file', urdf_path,
                    '-x', str(pos[0]),
                    '-y', str(pos[1]),
                    '-z', str(pos[2])
                ],
                output='screen'
            )
        )
        
    # Start our trajectory controller node
    controller_node = Node(
        package='multi_uav_sim',
        executable='uav_sim_node',
        output='screen'
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo_cmd)
    for node in spawn_nodes:
        ld.add_action(node)
    ld.add_action(controller_node)
    
    return ld
