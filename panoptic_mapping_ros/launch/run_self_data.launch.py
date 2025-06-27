from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # 声明参数
    namespace_arg = DeclareLaunchArgument('namespace', default_value='data')
    use_rio_arg = DeclareLaunchArgument('use_rio', default_value='false')
    use_detectron_arg = DeclareLaunchArgument('use_detectron', default_value='true')
    visualize_arg = DeclareLaunchArgument('visualize', default_value='true')

    base_path_arg = DeclareLaunchArgument('base_path', default_value='/mnt/data/3d-lidar/semantic/self_collect/for_panoptic_mapping_3')
    play_rate_arg = DeclareLaunchArgument('play_rate', default_value='1.0')
    wait_time_arg = DeclareLaunchArgument('wait_time', default_value='0')
    max_frames_arg = DeclareLaunchArgument('max_frames', default_value='1000')

    data_path_arg = DeclareLaunchArgument('data_path', default_value='/home/lukas/Documents/Datasets/3RScan')
    scene_id_arg = DeclareLaunchArgument('scene_id', default_value='0')
    scan_id_arg = DeclareLaunchArgument('scan_id', default_value='0')
    rio_play_rate_arg = DeclareLaunchArgument('rio_play_rate', default_value='20')

    config_arg = DeclareLaunchArgument('config', default_value='realsense_owlvit_sam.yaml')
    shutdown_when_finished_arg = DeclareLaunchArgument('shutdown_when_finished', default_value='false')

    load_map_arg = DeclareLaunchArgument('load_map', default_value='false')
    load_file_arg = DeclareLaunchArgument('load_file', default_value='/mnt/data/3d-lidar/semantic/panoptic_mapping/test_result/run1.panmap')

    # 条件控制参数
    wait_arg = DeclareLaunchArgument('wait', default_value='true', condition=IfCondition(LaunchConfiguration('load_map')))

    # 包路径查找
    panoptic_mapping_utils_pkg = FindPackageShare('panoptic_mapping_utils')
    panoptic_mapping_ros_pkg = FindPackageShare('panoptic_mapping_ros')
    print("panoptic_mapping_ros_pkg: ", panoptic_mapping_ros_pkg.find("panoptic_mapping_ros"))

    # 包含子 launch 文件
    play_flat_dataset_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            panoptic_mapping_utils_pkg,
            '/launch/play_flat_dataset.launch.py'
        ]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_detectron', LaunchConfiguration('use_detectron')),
            ('base_path', LaunchConfiguration('base_path')),
            ('play_rate', LaunchConfiguration('play_rate')),
            ('wait_time', LaunchConfiguration('wait_time')),
            ('max_frames', LaunchConfiguration('max_frames'))
        ],
        condition=UnlessCondition(LaunchConfiguration('use_rio'))
    )

    play_rio_dataset_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            panoptic_mapping_utils_pkg,
            '/launch/play_rio_dataset.launch.py'
        ]),
        launch_arguments=[
            ('namespace', LaunchConfiguration('namespace')),
            ('use_detectron', LaunchConfiguration('use_detectron')),
            ('data_path', LaunchConfiguration('data_path')),
            ('scene_id', LaunchConfiguration('scene_id')),
            ('scan_id', LaunchConfiguration('scan_id')),
            ('rio_play_rate', LaunchConfiguration('rio_play_rate'))
        ],
        condition=IfCondition(LaunchConfiguration('use_rio'))
    )

    # Mapper 节点
    mapper_node = Node(
        package='panoptic_mapping_ros',
        executable='panoptic_mapper_node',
        name='panoptic_mapper',
        output='screen',
        # prefix=['xterm -e gdb -ex run --args'],
        parameters=[
            {'config_path': PathJoinSubstitution([panoptic_mapping_ros_pkg, 'config/mapper', LaunchConfiguration('config')])}
        ],
        remappings=[
            ('color_image_in', [LaunchConfiguration('namespace'), '/color_image']),
            ('depth_image_in', [LaunchConfiguration('namespace'), '/depth_image']),
            ('segmentation_image_in', [LaunchConfiguration('namespace'), '/segmentation_image']),
            ('labels_in', [LaunchConfiguration('namespace'), '/segmentation_labels'])
        ],
        on_exit=Shutdown() if LaunchConfiguration('shutdown_when_finished') == 'true' else []
    )

    # Map loader 节点
    map_loader_node = Node(
        package='panoptic_mapping_utils',
        executable='map_loader.py',
        name='map_loader',
        output='screen',
        parameters=[
            {'path': LaunchConfiguration('load_file')},
            {'srv_name': '/panoptic_mapper/load_map'},
            {'delay': '0.1'}
        ],
        condition=IfCondition(LaunchConfiguration('load_map'))
    )

    # RVIZ 可视化节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', PathJoinSubstitution([panoptic_mapping_ros_pkg, 'config/rviz/devel.rviz'])],
        condition=IfCondition(LaunchConfiguration('visualize'))
    )

    return LaunchDescription([
        # 参数声明
        namespace_arg,
        use_rio_arg,
        use_detectron_arg,
        visualize_arg,
        base_path_arg,
        play_rate_arg,
        wait_time_arg,
        max_frames_arg,
        data_path_arg,
        scene_id_arg,
        scan_id_arg,
        rio_play_rate_arg,
        config_arg,
        shutdown_when_finished_arg,
        load_map_arg,
        load_file_arg,
        wait_arg,

        # 子 launch 文件
        play_flat_dataset_launch,
        play_rio_dataset_launch,

        # 主要节点
        mapper_node,
        map_loader_node,
        rviz_node
    ])