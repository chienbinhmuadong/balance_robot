import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    pkg_name = 'my_ros2_package'
    pkg_share = get_package_share_directory(pkg_name)
    config_file_path = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    models_path = os.path.join(pkg_share, 'models') # duong dan toi thu muc models

    # bao cho Gazebo cho tim file .stl
    set_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        models_path
    )

    # mo Gazebo voi world empty
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        # '-r' de Gazebo tu dong play khong can nhan nut
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    # oc va nap (spawn) file model .sdf vao Gazebo
    sdf_file = os.path.join(models_path, 'simple_robot', 'model.sdf')
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', sdf_file,
            '-name', 'simple_robot',
            '-z', '0.1'  #tha robot cach mat dat 0.1m
        ],
        output='screen'
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': config_file_path}],
        output='screen'
    )

    controller_node = Node(
        package='my_ros2_package',
        executable='controller',
        # parameters=[{'use_sim_time':True}],
        output='screen'
    )
    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        set_model_path,
        gazebo_launch,
        spawn_robot,
        bridge_node,
        controller_node
    ])