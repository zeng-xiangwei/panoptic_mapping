from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os


def generate_launch_description():
    # 声明参数
    use_rio_arg = DeclareLaunchArgument('use_rio', default_value='false')
    use_detectron_arg = DeclareLaunchArgument('use_detectron',
                                              default_value='true')
    visualize_arg = DeclareLaunchArgument('visualize', default_value='true')

    config_arg = DeclareLaunchArgument(
        'config', default_value='realsense_owlvit_sam_online.yaml')
    shutdown_when_finished_arg = DeclareLaunchArgument(
        'shutdown_when_finished', default_value='false')

    load_map_arg = DeclareLaunchArgument('load_map', default_value='false')
    load_file_arg = DeclareLaunchArgument(
        'load_file',
        default_value=
        '/home/diana/Code/panoptic_mapping_ws/data/map.panmap')

    # 包路径查找
    panoptic_mapping_ros_pkg = FindPackageShare('panoptic_mapping_ros')
    print("panoptic_mapping_ros_pkg: ",
          panoptic_mapping_ros_pkg.find("panoptic_mapping_ros"))

    # Mapper 节点
    mapper_node = Node(
        package='panoptic_mapping_ros',
        executable='panoptic_mapper_node',
        name='panoptic_mapper',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[{
            'config_path':
            PathJoinSubstitution([
                panoptic_mapping_ros_pkg, 'config/mapper',
                LaunchConfiguration('config')
            ]),
            'load_map': LaunchConfiguration('load_map'),
            'load_file': LaunchConfiguration('load_file'),
            # 'log_dir': os.path.expanduser('~/.ros/log')
        }],
        remappings=[
            ('color_image_in', '/camera/camera/color/image_raw'),
            ('depth_image_in', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('segmentation_image_in', '/segmentation_mask'),
            ('labels_in', '/detection_result'),
            ('vln_map_update', '/vln/semantic_map'),
            ('request_vl_processing', '/vln_slow/image_description')
        ],
        on_exit=Shutdown()
        if LaunchConfiguration('shutdown_when_finished') == 'true' else [])

    # RVIZ 可视化节点
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz',
                     output='screen',
                     arguments=[
                         '-d',
                         PathJoinSubstitution([
                             panoptic_mapping_ros_pkg, 'config/rviz/devel.rviz'
                         ])
                     ],
                     condition=IfCondition(LaunchConfiguration('visualize')))

    return LaunchDescription([
        # 参数声明
        use_rio_arg,
        use_detectron_arg,
        visualize_arg,
        config_arg,
        shutdown_when_finished_arg,
        load_map_arg,
        load_file_arg,

        # 主要节点
        mapper_node,
        rviz_node
    ])
