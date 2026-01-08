from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'foundation_pose_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        

        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes/model2/model'), glob('meshes/model2/model/*')),
        (os.path.join('share', package_name, 'meshes/camera'), glob('meshes/camera/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo@todo.todo',
    description='Foundation Pose TF - Pick and Place Pipeline',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
             'pose_to_tf = foundation_pose_tf.pose_to_tf:main',
             'object_marker = foundation_pose_tf.object_marker:main',
             'camera_marker = foundation_pose_tf.camera_marker:main',
             'detected_object_pose_pub = foundation_pose_tf.detected_object_pose_pub:main',
             'model2_marker = foundation_pose_tf.model2_marker:main',
             'ghost_marker = foundation_pose_tf.ghost_marker:main',
             'intelligent_grasper = foundation_pose_tf.intelligent_grasper:main',
             'scene_manager = foundation_pose_tf.scene_manager:main',
             'grasp_candidates_markers = foundation_pose_tf.grasp_candidates_markers:main',
             'grasp_ghost_mirror = foundation_pose_tf.grasp_ghost_mirror:main',
             'ghost_table_filter = foundation_pose_tf.ghost_table_filter:main',
             'ghost_to_detected_mirror = foundation_pose_tf.ghost_to_detected_mirror:main',
             'grasp_plan_only_selector = foundation_pose_tf.grasp_plan_only_selector:main',
             'detected_object_snapshot = foundation_pose_tf.detected_object_snapshot:main',
             'ghost_pose_solucion_publisher = foundation_pose_tf.ghost_pose_solucion_publisher:main',

        ],
    },
)