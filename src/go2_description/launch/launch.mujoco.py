import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    package_description = context.launch_configurations['pkg_description']
    pkg_path = os.path.join(get_package_share_directory(package_description))

    # Launch arguments win; otherwise the UNITREE_NETWORK_INTERFACE /
    # UNITREE_DOMAIN environment variables (domain 0 = real robot, 1 = Mujoco).
    env_domain = os.environ.get('UNITREE_DOMAIN', '1').strip()
    real_arg = context.launch_configurations['real_robot'].strip().lower()
    real_robot = real_arg == 'true' if real_arg else env_domain == '0'
    env_interface = os.environ.get('UNITREE_NETWORK_INTERFACE', '').strip()
    network_interface = (
        context.launch_configurations['network_interface'].strip()
        or (env_interface if real_robot else '')
        or 'lo')
    if real_robot and network_interface == 'lo':
        raise RuntimeError(
            "Real robot needs the Ethernet port wired to the Go2: set "
            "network_interface:=<port> or UNITREE_NETWORK_INTERFACE (see 'ip a', "
            "the port with a 192.168.123.x address)")
    print(f"[go2 launch] {'REAL ROBOT' if real_robot else 'Mujoco'}: "
          f"network_interface={network_interface}, "
          f"domain={'0' if real_robot else '1'}, "
          f"require_sport_mode_off={'true' if real_robot else 'false'}")

    xacro_file = os.path.join(pkg_path, 'xacro', 'robot.xacro')
    robot_description = xacro.process_file(
        xacro_file,
        mappings={
            'network_interface': network_interface,
            'domain': '0' if real_robot else '1',
            'require_sport_mode_off': 'true' if real_robot else 'false',
        },
    ).toxml()

    config_dir = os.path.join(pkg_path, 'config')
    # Real robot: go2_controllers_real.yaml overrides a few parameters of
    # go2_controllers.yaml (IMU calibration, command timeout, ...)
    param_files = [os.path.join(config_dir, 'go2_controllers.yaml')]
    if real_robot:
        param_files.append(os.path.join(config_dir, 'go2_controllers_real.yaml'))
    param_file_args = []
    for f in param_files:
        param_file_args += ["--param-file", f]

    rviz_config_file = os.path.join(get_package_share_directory(package_description), "config", "visualize_urdf.rviz")

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_ocs2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    # some difference to be tested
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'publish_frequency': 20.0,
                'use_tf_static': True,
                'robot_description': robot_description,
                'ignore_timestamp': True
            }
        ],
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=param_files,
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_publisher = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    state_estimator = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "state_estimator",
            "-c","/controller_manager",
            *param_file_args,
        ]
    )
    stand_up_controler = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "stand_up",
            "-c","/controller_manager",
            *param_file_args,
        ]
    )

    whole_body_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "whole_body_controller",
            "-c", "/controller_manager",
            *param_file_args,
            "--inactive"
        ]
    )

    nodes = [rviz] if context.launch_configurations['rviz'].lower() == 'true' else []
    return nodes + [
        controller_manager,
        robot_state_publisher,
        joint_state_publisher,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_publisher,
                on_exit=[state_estimator],
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=state_estimator,
                on_exit=[stand_up_controler, whole_body_controller],
            )
        ),
    ]


def generate_launch_description():
    pkg_description = DeclareLaunchArgument(
        'pkg_description',
        default_value='go2_description',
        description='package for robot description'
    )

    return LaunchDescription([
        pkg_description,
        DeclareLaunchArgument(
            'real_robot', default_value='',
            description='true: real Go2 (domain 0; run release_sport_mode first). '
                        'Default: true if UNITREE_DOMAIN=0, else false'),
        DeclareLaunchArgument(
            'network_interface', default_value='',
            description='Ethernet port wired to the Go2. '
                        'Default: UNITREE_NETWORK_INTERFACE, else lo'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Start RViz'),
        OpaqueFunction(function=launch_setup),
    ])