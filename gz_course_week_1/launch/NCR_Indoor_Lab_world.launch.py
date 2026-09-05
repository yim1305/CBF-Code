import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
def _prepend_path(new_path: str, env_var: str) -> str:
    existing = os.environ.get(env_var, '')
    parts = [p for p in [new_path, existing] if p]
    return ':'.join(parts)
def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='teleop',
        description=(
            "'teleop' (default): run teleop_keyboard yourself in another terminal. "
            "'spiral': starts spiral_tracker_node instead -- don't run both."
        ),
    )
    spiral_radial_rate_arg = DeclareLaunchArgument(
        'spiral_radial_rate',
        default_value='0.06',
        description="mode:=spiral only. radius(t) = spiral_radial_rate * t.",
    )
    spiral_angular_rate_arg = DeclareLaunchArgument(
        'spiral_angular_rate',
        default_value='0.24',
        description="mode:=spiral only. Spiral's angular rate (rad/s); tangential speed = radius*rate.",
    )
    cbf_arg = DeclareLaunchArgument(
        'cbf',
        default_value='true',
        description=(
            "'true' (default): cbf_safety_node filters /cmd_vel into /safe_cmd_vel, "
            "which drives the robot. 'false': cbf_safety_node isn't started and "
            "/cmd_vel drives the robot directly, unfiltered."
        ),
    )
    pkg_share = get_package_share_directory('gz_course_week_1')
    world_file = os.path.join(pkg_share, 'worlds', 'NCR_Indoor_Lab_world.sdf')
    models_dir = os.path.join(pkg_share, 'models')
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=_prepend_path(models_dir, 'GZ_SIM_RESOURCE_PATH')
    )
    set_sdf_path = SetEnvironmentVariable(
        name='SDF_PATH',
        value=_prepend_path(models_dir, 'SDF_PATH')
    )
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )
    core_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )
    drive_bridge_cbf_on = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/safe_cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        output='screen',
        condition=LaunchConfigurationEquals('cbf', 'true'),
    )
    drive_bridge_cbf_off = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/safe_cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
        remappings=[('/safe_cmd_vel', '/cmd_vel')],
        output='screen',
        condition=LaunchConfigurationEquals('cbf', 'false'),
    )
    cbf_safety_node = Node(
        package='gz_course_week_1',
        executable='cbf_safety_node',
        output='screen',
        condition=LaunchConfigurationEquals('cbf', 'true'),
    )
    ground_truth_pose_node = Node(
        package='gz_course_week_1',
        executable='ground_truth_pose',
        output='screen',
    )
    trajectory_recorder_node = Node(
        package='gz_course_week_1',
        executable='trajectory_recorder',
        output='screen',
    )
    spiral_tracker_node = Node(
        package='gz_course_week_1',
        executable='spiral_tracker',
        output='screen',
        parameters=[{
            'radial_rate': ParameterValue(LaunchConfiguration('spiral_radial_rate'), value_type=float),
            'angular_rate': ParameterValue(LaunchConfiguration('spiral_angular_rate'), value_type=float),
        }],
        condition=LaunchConfigurationEquals('mode', 'spiral'),
    )
    mode_notice = LogInfo(
        msg=['NCR_Indoor_Lab_world: mode=', LaunchConfiguration('mode'),
             " -- 'teleop' means run teleop_keyboard yourself; 'spiral' means "
             "spiral_tracker_node is publishing /cmd_vel and teleop_keyboard "
             "must NOT be run alongside it."]
    )
    cbf_notice = LogInfo(
        msg=['NCR_Indoor_Lab_world: cbf=', LaunchConfiguration('cbf'),
             " -- 'true' means the robot is driven off CBF-filtered "
             "/safe_cmd_vel; 'false' means cbf_safety_node is not running "
             "and the robot is driven directly off unfiltered /cmd_vel."]
    )
    return LaunchDescription([
        mode_arg,
        spiral_radial_rate_arg,
        spiral_angular_rate_arg,
        cbf_arg,
        mode_notice,
        cbf_notice,
        set_gz_resource_path,
        set_sdf_path,
        gz_launch,
        core_bridge,
        drive_bridge_cbf_on,
        drive_bridge_cbf_off,
        ground_truth_pose_node,
        cbf_safety_node,
        spiral_tracker_node,
        trajectory_recorder_node,
    ])