import os
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory 
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('hw_insight')

    # RVIZ2显示配置
    rviz_path = os.path.join(pkg_share, 'rviz/Track.rviz')
    rviz_node = launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path]
    )

    # 启动AirSim节点
    airsim_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('airsim_ros_pkgs'), 'launch/airsim_node.launch.py')
        )
    )

    yolo_model = DeclareLaunchArgument(
        "yolo_model",
        default_value='yolov8n.pt')

    # 启动YOLO
    yolo_node1 = launch_ros.actions.Node(
            package='hw_insight',
            executable='yolo',
            name='yolo',
            parameters=[{
                'yolo_model': LaunchConfiguration('yolo_model'),
                'camera_name': 'CameraDepth1'
            }]
    )
    yolo_node2 = launch_ros.actions.Node(
            package='hw_insight',
            executable='yolo',
            name='yolo',
            parameters=[{
                'yolo_model': LaunchConfiguration('yolo_model'),
                'camera_name': 'CameraDepth2'
            }]
    )
    yolo_node3 = launch_ros.actions.Node(
            package='hw_insight',
            executable='yolo',
            name='yolo',
            parameters=[{
                'yolo_model': LaunchConfiguration('yolo_model'),
                'camera_name': 'CameraDepth3'
            }]
    )
    yolo_vehicle_node = launch_ros.actions.Node(
        package='hw_insight',
        executable='yolo_vehicle',  # 修改后的车辆检测节点
        name='yolo_vehicle',
        parameters=[{
            'yolo_model': LaunchConfiguration('yolo_model'),
            'camera_name': 'CameraDepth1'
        }]
    )

    # 启动追踪控制
    track_node = launch_ros.actions.Node(
            package='hw_insight',
            executable='track',
            name='track'
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # Create the launch description and populate
    ld.add_action(yolo_model)
    ld.add_action(airsim_node_launch)
    ld.add_action(yolo_node1)
    #ld.add_action(yolo_node2)
    #ld.add_action(yolo_node3)
    ld.add_action(rviz_node)
    ld.add_action(track_node)

    return ld

