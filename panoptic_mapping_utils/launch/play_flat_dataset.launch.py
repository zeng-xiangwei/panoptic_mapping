# Converted ROS 2 launch file for playing flat dataset

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    base_path_arg = DeclareLaunchArgument(
        'base_path',
        default_value='/mnt/data/3d-lidar/semantic/self_collect/for_panoptic_mapping_3',
        description='Base path of the dataset'
    )
    play_rate_arg = DeclareLaunchArgument(
        'play_rate',
        default_value='1.0',
        description='Playback rate'
    )
    wait_arg = DeclareLaunchArgument(
        'wait',
        default_value='false',
        description='Wait for input before starting playback'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='data',
        description='Namespace for remapped topics'
    )
    use_detectron_arg = DeclareLaunchArgument(
        'use_detectron',
        default_value='true',
        description='Whether to use Detectron for segmentation'
    )
    max_frames_arg = DeclareLaunchArgument(
        'max_frames',
        default_value='1000000000',
        description='Maximum number of frames to play'
    )
    global_frame_name_arg = DeclareLaunchArgument(
        'global_frame_name',
        default_value='world',
        description='Name of the global frame'
    )
    sensor_frame_name_arg = DeclareLaunchArgument(
        'sensor_frame_name',
        default_value='depth_camera',
        description='Name of the sensor frame'
    )
    static_transform_arg = DeclareLaunchArgument(
        'static_transform',
        default_value='0,0,1,0,-1,0,0,0,0,-1,0,0,0,0,0,1',
        description='Static transform matrix'
    )
    add_labels_name_arg = DeclareLaunchArgument(
        'add_labels_name',
        default_value='true',
        description='Add labels name'
    )
    labels_cvs_path_arg = DeclareLaunchArgument(
        'labels_cvs_path',
        default_value='/mnt/data/3d-lidar/semantic/self_collect/realsense_labels.csv',
        description='Path to CSV file containing label names'
    )

    # Define the node
    data_player_node = Node(
        package='panoptic_mapping_utils',
        executable='flat_data_player.py',
        name='flat_data_player',
        output='screen',
        parameters=[
            {'data_path': LaunchConfiguration('base_path')},
            {'play_rate': LaunchConfiguration('play_rate')},
            {'use_detectron': LaunchConfiguration('use_detectron')},
            {'global_frame_name': LaunchConfiguration('global_frame_name')},
            {'sensor_frame_name': LaunchConfiguration('sensor_frame_name')},
            {'wait': LaunchConfiguration('wait')},
            {'max_frames': LaunchConfiguration('max_frames')},
            {'static_transform': LaunchConfiguration('static_transform')},
            {'add_labels_name': LaunchConfiguration('add_labels_name')},
            {'labels_cvs_path': LaunchConfiguration('labels_cvs_path')}
        ],
        remappings=[
            ('color_image', [LaunchConfiguration('namespace'), '/color_image']),
            ('depth_image', [LaunchConfiguration('namespace'), '/depth_image']),
            ('id_image', [LaunchConfiguration('namespace'), '/segmentation_image']),
            ('labels', [LaunchConfiguration('namespace'), '/segmentation_labels']),
            ('pose', [LaunchConfiguration('namespace'), '/pose'])
        ]
    )

    return LaunchDescription([
        base_path_arg,
        play_rate_arg,
        wait_arg,
        namespace_arg,
        use_detectron_arg,
        max_frames_arg,
        global_frame_name_arg,
        sensor_frame_name_arg,
        static_transform_arg,
        add_labels_name_arg,
        labels_cvs_path_arg,
        data_player_node
    ])