import math
import os
import subprocess
import random
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import AppendEnvironmentVariable, ExecuteProcess, TimerAction, OpaqueFunction

package_name = 'ros2_ur5_interface'

def spawn_block(context, *args, **kwargs):
    block_numbers = ["1", "2", "3"]  # Add more block numbers as needed
    #block_numbers = ["1"] # for testing
    block_types = ["X1-Y1-Z2", "X1-Y2-Z2", "X1-Y4-Z2", "X1-Y2-Z1", "X1-Y3-Z2-FILLET", "X2-Y2-Z2-FILLET", "X1-Y2-Z2-CHAMFER", "X1-Y3-Z2", "X2-Y2-Z2", "X1-Y2-Z2-TWINFILLET", "X1-Y4-Z1"]
    
    instances_cmds = []

    for block_number in block_numbers:
        #block_type = random.choice(block_types)
        block_type = block_types[int(block_number) - 1]
        
        # Paths
        xacro_file = os.path.join(get_package_share_directory(package_name), 'models', 'block.urdf.xacro')
        urdf_file = os.path.join(get_package_share_directory(package_name), 'models', f'block_{block_number}.urdf')
        sdf_file = os.path.join(get_package_share_directory(package_name), 'models', f'block_{block_number}.sdf')

        # Generate URDF from Xacro
        try:
            xacro_command = [
                FindExecutable(name="xacro").perform(context),
                xacro_file,
                f"block_name:={block_number}",
                f"block_type:={block_type}",
            ]
            urdf_output = subprocess.check_output(xacro_command, text=True)
            with open(urdf_file, 'w') as urdf_fp:
                urdf_fp.write(urdf_output)

        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error generating URDF: {e}")

        # Convert URDF to SDF -> URDF (Unified Robot Description Format) file to an SDF (Simulation Description Format)
        try:
            sdf_command = [
                FindExecutable(name="gz").perform(context),
                "sdf",
                "-p",
                urdf_file,
            ]
            sdf_output = subprocess.check_output(sdf_command, text=True)
            # Modify SDF to include the IMU sensor
            sdf_lines = sdf_output.splitlines()
            sensor_block = f"""
            <plugin
                filename="ignition-gazebo-pose-publisher-system"
                name="ignition::gazebo::systems::PosePublisher">
                <publish_model_pose>true</publish_model_pose>
                <publish_nested_model_pose>true</publish_nested_model_pose>
                <use_pose_vector_msg>true</use_pose_vector_msg>
                <update_frequency>100.0</update_frequency>
            </plugin>
            """
            # Append the sensor to the appropriate location
            insert_index = next(
                (i for i, line in enumerate(sdf_lines) if "</model>" in line), len(sdf_lines) - 1
            )
            sdf_lines.insert(insert_index, sensor_block)
            modified_sdf_output = "\n".join(sdf_lines)
            with open(sdf_file, 'w') as sdf_fp:
                sdf_fp.write(modified_sdf_output)

        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Error converting URDF to SDF: {e}")

        print(f"Successfully generated URDF at {urdf_file}")
        print(f"Successfully generated SDF at {sdf_file}")

        # Block robot state publisher node
        block_robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=f'block_{block_number}',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_output}]
        )
        instances_cmds.append(block_robot_state_publisher_node)
        
        x_min = 0.05
        x_max = 0.90
        y_min = 0.25
        y_max = 0.75
        
        
        # Generate random position within the desk limits 
        # x_pos = random.uniform(x_min, x_max) 
        # y_pos = random.uniform(y_min, y_max) 
        # z_pos = 0.88
        #random_rotation = random.uniform(0, 45)

        # Define desk zones for each block # keith version
        zones = [
            {'x_min': 0.05, 'x_max': 0.25, 'y_min': 0.4, 'y_max': 0.75},
            {'x_min': 0.30, 'x_max': 0.50, 'y_min': 0.40, 'y_max': 0.75},
            {'x_min': 0.55, 'x_max': 0.75, 'y_min': 0.25, 'y_max': 0.75}
        ]

        # Step 1: Select ONE random zone for all blocks
        selected_zone = zones[2] #random.choice(zones)

        # Step 2: Place all blocks randomly within this zone
        for block_number in block_numbers:
            # Generate a random position within the selected zone
            x_pos = random.uniform(selected_zone['x_min'], selected_zone['x_max'])
            y_pos = random.uniform(selected_zone['y_min'], selected_zone['y_max'])
            z_pos = 0.88  # Fixed height

            spawn_block = Node( 
                package='ros_gz_sim', 
                executable='create', 
                arguments=[ 
                    '-name', f"block_{block_number}", 
                    '-file', sdf_file, 
                    '-x', str(x_pos), 
                    '-y', str(y_pos), 
                    '-z', str(z_pos), 
                    '-R', '0',  
                    '-P', '0',  
                    '-Y', str(math.pi/2)  # Fixed rotation on Z
                ], 
                output='screen', 
            ) 
            instances_cmds.append(spawn_block)

    return instances_cmds



def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
            default_value="ur5e",
        )
    )

    ur_type = LaunchConfiguration("ur_type")
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','empty.world')
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz', 'ur5.rviz')
    desk_urdf = Command([PathJoinSubstitution([FindExecutable(name='xacro')])," ",PathJoinSubstitution([FindPackageShare(package_name), "models", "desk.urdf.xacro"])])
    camera_sdf = os.path.join(get_package_share_directory(package_name), 'models', 'camera.sdf')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(package_name), "models", "ur_gz.urdf.xacro"]),
            " ",
            "safety_limits:=", "true",
            " ",
            "safety_pos_margin:=", "0.15",
            " ",
            "safety_k_position:=", "20",
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=", "",
            " ",
            "simulation_controllers:=",
            PathJoinSubstitution([FindPackageShare(package_name), "config", "ur_controllers.yaml"]),
        ]
    )

    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory(package_name), 'models') +
        ':' +
        os.path.dirname(get_package_share_directory('ur_description'))
    )

    # Fixed transform broadcaster
    fixed_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'desk', 'default']
    )

    # Fixed transform broadcaster for the camera
    fixed_camera_tf_broadcast = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['-0.5', '0.5', '1.2', '-0.06', '0.4', '0.0', 'desk', 'camera_rgb_frame']
    )

    # UR robot state publisher node
    ur_robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Desk robot state publisher node
    desk_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace='desk',
        name='robot_state_publisher',
        parameters=[{'robot_description': desk_urdf}]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    joint_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': ['-r -s ', world_file ], 'on_exit_shutdown': 'true'}.items()
        #                              -r -s -v4
    )
    
    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'camera',
            '-file', camera_sdf,
            '-x', '-0.5',
            '-y', '0.5',
            '-z', '1.2',
            '-R', '0.0',
            '-P', '0.4',
            '-Y', '-0.06',
        ],
        output='screen',
    )

    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', ur_type,
            '-string', robot_description_content,
        ],
        output='screen',
    )

    spawn_desk = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "desk",
            '-string', desk_urdf,
        ],
        output='screen',
    )

    activate_gripper = Node(
        package='ros2_ur5_interface',
        executable='gripper_service',
        output='screen',
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        'params',
        'ur5_bridge.yaml'
    )

    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    gazebo_ros_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw/image'],
        output='screen',
    )

    # RViz2 node
    rviz2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_file],
                    output='screen'
                ),
            ],
        ),
    )

    # Return the LaunchDescription
    return LaunchDescription([
        *declared_arguments,
        set_env_vars,
        fixed_tf_broadcast,
        fixed_camera_tf_broadcast,
        desk_state_publisher_node,
        ur_robot_state_publisher_node,
        OpaqueFunction(function=spawn_block),
        joint_state_broadcaster_spawner,
        joint_controller_spawner,
        gripper_controller_spawner,
        gazebo_launch,
        spawn_camera,
        spawn_ur5,
        spawn_desk,
        activate_gripper,
        gazebo_ros_bridge,
        gazebo_ros_image_bridge,
        rviz2,
    ])
