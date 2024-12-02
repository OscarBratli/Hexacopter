from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

# Function to generate the launch description
def generate_launch_description():

    # Set the package name and path to the package
    package_name = 'my_hexacopter'
    package_path = get_package_share_directory(package_name)
    robot_arm_xacro = os.path.join(package_path, 'urdf', 'robot_arm.xacro')

    # Process the xacro files to generate the robot description
    robot_description = Command(['xacro ', robot_arm_xacro])

    # Load controller configuration file
    controller_config = os.path.join(package_path, 'config', 'controller_config.yaml')

    # Define a node to publish the robot's state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
        arguments=[]
    )

    # Define a node to publish a static transformation between "map" and "base_link"
    node_tf = Node(
        package="tf2_ros",  # The package where the static_transform_publisher is found
        executable="static_transform_publisher",  # The executable for the node
        arguments=["0", "0", "0", "0", "0", "0", "map", "base_link"]  # Static transform values (position and orientation)
    )

    # Include the Gazebo launch file to start Gazebo simulation with the custom world
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'pause': 'false'  # Start Gazebo in a paused state
        }.items()
    )

    # Define a node to spawn the robot in the Gazebo environment, 2 meters up in the air
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description', 
            '-entity', 'my_hexacopter', 
            '-x', '0', '-y', '0', '-z', '0',],
        output='screen'
    )   

    # Define the list of controllers to be spawned
    controllers = [
        'joint_state_broadcaster',  # Broadcasts the joint states
        'joint_position_controller' # Controls the joint positions
    ]

    # Event handlers to spawn controllers after the robot is spawned
    delayed_controller_spawners = []
    for controller in controllers:
        delayed_controller_spawners.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,  # Triggered when the robot is spawned
                    on_exit=[
                        TimerAction(
                            period=5.0,  # Delay before spawning the controller (in seconds)
                            actions=[
                                Node(
                                    package='controller_manager',
                                    executable='spawner.py',
                                    arguments=[
                                        controller,],
                                    output='screen'
                                )
                            ]
                        )
                    ]
                )
            )
        )

    # Event handler to launch rqt_gui after controllers are spawned
    rqt_gui_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,  # Triggered when the robot is spawned
            on_exit=[
                TimerAction(
                    period=10.0,  # Delay to ensure controllers are up before launching rqt_gui
                    actions=[
                        Node(
                            package='rqt_gui',  # The package where rqt_gui is found
                            executable='rqt_gui',  # The executable for the node
                            output='screen'  # Output will be printed to the screen
                        )
                    ]
                )
            ]
        )
    )

    # Return the launch description, including all nodes and event handlers
    return LaunchDescription([
        node_robot_state_publisher,  # Node to publish robot state
        node_tf,  # Node to publish static transform
        start_gazebo,  # Include Gazebo launch file
        spawn_entity,  # Node to spawn the robot
        *delayed_controller_spawners,  # Event handlers to spawn controllers after robot is spawned
        #rqt_gui_node  # Uncomment this line to include rqt_gui launch
    ])
