import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 1. Đường dẫn file cấu hình params.yaml
    pkg_share = FindPackageShare('esekf19_nav')
    default_params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])

    # 2. Các tham số dòng lệnh Launch Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Sử dụng thời gian mô phỏng Gazebo (/clock)'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Đường dẫn tuyệt đối đến file tham số YAML'
    )

    # 3. Node thuật toán ES-EKF 19 trạng thái
    esekf_node = Node(
        package='esekf19_nav',
        executable='esekf19_node',
        name='esekf19_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Có thể remap topic tại đây nếu không sửa params.yaml
            # ('/imu/data', '/uuv/imu'),
            # ('/dvl/velocity', '/uuv/dvl/twist'),
        ]
    )

    # =========================================================================
    # TODO CHO PARTNER GAZEBO:
    # Để khởi động đồng thời mô phỏng Gazebo (UUV Simulator / Stonefish / VRX),
    # uncomment và include launch file của mô phỏng tại đây:
    #
    # from launch.actions import IncludeLaunchDescription
    # from launch.launch_description_sources import PythonLaunchDescriptionSource
    #
    # gazebo_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('my_gazebo_auv_pkg'), # Tên gói mô phỏng Gazebo
    #             'launch',
    #             'auv_ocean_world.launch.py'            # File launch Gazebo
    #         ])
    #     ]),
    #     launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    # )
    # =========================================================================

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        LogInfo(msg="Đang khởi động Node ES-EKF 19-Trạng thái (GEO-FOG 3D + DVL Calibration)..."),
        esekf_node,
        # gazebo_sim_launch, # <--- Bỏ comment dòng này sau khi tích hợp gói Gazebo
    ])
