import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_civiclens = get_package_share_directory('civiclens_gazebo')

    models_path = os.path.join(pkg_civiclens, 'models')
    world = os.path.join(pkg_civiclens, 'worlds', 'civiclens_indian_city.world')

    plugin_path = os.path.join(pkg_civiclens, '..', '..', 'lib', 'civiclens_gazebo')

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_path),
        SetEnvironmentVariable(name='SDF_PATH', value=models_path),
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path),
        SetEnvironmentVariable(name='GAZEBO_PLUGIN_PATH', value=plugin_path),
        
        # Start Gazebo Sim with CivicLens City World
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world}'}.items()
        ),

        # Bridge for ROS 2 <-> Gazebo topics
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/civiclens/truck_command@std_msgs/msg/String]gz.msgs.StringMsg',
                '/civiclens/truck_state@std_msgs/msg/String[gz.msgs.StringMsg',
                '/civiclens/truck_telemetry@std_msgs/msg/String[gz.msgs.StringMsg',
            ],
            output='screen'
        ),

        # 1. Smart Dustbin AI Vision & IoT Node
        Node(
            package='civiclens_gazebo',
            executable='civiclens_ai_vision.py',
            name='civiclens_ai_vision',
            output='screen'
        ),

        # 2. Waste Segregation & Fleet Manager
        Node(
            package='civiclens_gazebo',
            executable='waste_manager',
            name='waste_manager',
            output='screen'
        ),

        # 3. Central Mission Autonomy & Waypoint Dispatcher
        Node(
            package='civiclens_gazebo',
            executable='civiclens_autonomy',
            name='civiclens_autonomy',
            output='screen'
        ),

        # 4. Live Operations & Telemetry Web Dashboard (Port 8080)
        Node(
            package='civiclens_gazebo',
            executable='civiclens_dashboard.py',
            name='civiclens_dashboard',
            output='screen'
        )
    ])

