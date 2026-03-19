from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
import os


def create_log_dir(context, *args, **kwargs):
    """确保日志目录存在"""
    log_dir = LaunchConfiguration('log_dir').perform(context)
    if log_dir:  # 仅当 log_dir 不为空时创建目录
        os.makedirs(log_dir, exist_ok=True)
    return []


def generate_launch_description():
    # 声明参数
    use_rio_arg = DeclareLaunchArgument('use_rio', default_value='false')
    use_detectron_arg = DeclareLaunchArgument('use_detectron',
                                              default_value='true')
    visualize_arg = DeclareLaunchArgument('visualize', default_value='true')
    use_visulizer_bridge_arg = DeclareLaunchArgument('use_visulizer_bridge', default_value='true')

    config_arg = DeclareLaunchArgument(
        'config', default_value='realsense_owlvit_sam_online.yaml')
    shutdown_when_finished_arg = DeclareLaunchArgument(
        'shutdown_when_finished', default_value='false')

    load_map_arg = DeclareLaunchArgument('load_map', default_value='false')
    load_file_arg = DeclareLaunchArgument(
        'load_file',
        default_value=
        '/home/xiangweizeng/dataBag/panoptic_mapping/h11_02#/1217/map_floor2_qwen.panmap')
    
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='')

    # 包路径查找
    panoptic_mapping_ros_pkg = FindPackageShare('panoptic_mapping_ros')

    # Mapper 节点
    mapper_node = Node(
        package='panoptic_mapping_ros',
        executable='panoptic_mapper_node',
        name='panoptic_mapper',
        output='screen',
        # prefix=['gnome-terminal -- gdb -ex run --args'],
        parameters=[{
            'config_path':
            PathJoinSubstitution([
                panoptic_mapping_ros_pkg, 'config/mapper',
                LaunchConfiguration('config')
            ]),
            'load_map': LaunchConfiguration('load_map'),
            'load_file': LaunchConfiguration('load_file'),
            'log_dir': LaunchConfiguration('log_dir')
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


    # 与软件交互的节点
    visulizer_bridge_node = Node(
        package='panoptic_mapping_ros',
        executable='visulizer_bridge_node',
        name='visulizer_bridge_node',
        output='screen',
        # prefix=['gnome-terminal -- gdb -ex run --args'],
        parameters=[{
            'log_dir': LaunchConfiguration('log_dir')
        }],
        remappings=[
            ('visualization/submaps/mesh', '/visualization/submaps/mesh'),
            ('/single_tsdf_for_undetected/visualization/submaps/mesh', '/single_tsdf_for_undetected/visualization/submaps/mesh'),
            ('visualization/converted_mesh', '/visualization/converted_mesh'),
        ],
        condition=IfCondition(LaunchConfiguration('use_visulizer_bridge')))

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
        use_visulizer_bridge_arg,
        config_arg,
        shutdown_when_finished_arg,
        load_map_arg,
        load_file_arg,
        log_dir_arg,

        # 确保日志目录存在
        OpaqueFunction(function=create_log_dir),

        # 主要节点
        mapper_node,
        visulizer_bridge_node,
        rviz_node
    ])