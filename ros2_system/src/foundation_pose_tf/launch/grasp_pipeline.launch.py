from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'foundation_pose_tf'

    return LaunchDescription([

        # 1 Scene (mesa + objeto + cámara)
        Node(
            package=package_name,
            executable='scene_manager',
            name='scene_manager',
            output='screen'
        ),

        # 2 Intelligent Grasper → solution_0 … solution_7
        Node(
            package=package_name,
            executable='intelligent_grasper',
            name='intelligent_grasper',
            output='screen'
        ),

        # 3 Filtro geométrico inicial (mesa en detected_object)
        Node(
            package=package_name,
            executable='grasp_candidates_markers',
            name='grasp_geometric_filter',
            output='screen'
        ),

        # 4 Proyección de grasps a ghost_pose
        Node(
            package=package_name,
            executable='grasp_ghost_mirror',
            name='grasp_ghost_mirror',
            output='screen'
        ),

        # 5 Filtro mesa en ghost_pose
        Node(
            package=package_name,
            executable='ghost_table_filter',
            name='ghost_table_filter',
            output='screen',
            parameters=[{
                'table_z': 0.0,
                'clearance': 0.003,
                'axis': 'x',              # eje real del grasp marker
                'use_marker_scale': True,
                'fallback_length': 0.06,
                'debug': False
            }]
        ),

        # 6 Ghost → Detected (grasps finales válidos)
        Node(
            package=package_name,
            executable='ghost_to_detected_mirror',
            name='ghost_to_detected_mirror',
            output='screen'
        ),

        # 7 tcp_link (necesario para MoveIt)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tcp_virtual_frame',
            arguments=[
                '0.0', '0.0', '0.07',
                '0', '0', '0',
                'gripper_link', 'tcp_link'
            ]
        ),

        # 8 Snapshot del objeto (Congela la posición para el Pick and Place) #AGREGADO 
        Node(
            package=package_name,
            executable='detected_object_snapshot', # Asegúrate que este sea el nombre en tu setup.py
            name='detected_object_snapshot',
            output='screen'
        ),
        
    ])
