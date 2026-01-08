from setuptools import find_packages, setup

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Necesario para que ROS2 "vea" el paquete
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # Instalar package.xml
        ('share/' + package_name, ['package.xml']),

        # 🔥 Instalar launch files (AQUÍ ESTABA EL PROBLEMA)
        ('share/' + package_name + '/launch', [
            'launch/lite6_full.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugo',
    maintainer_email='hugo@todo.todo',
    description='Control del gripper Lite6 vía WebSocket + simulación.',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lite6_gripper_ws = gripper_control.lite6_gripper_ws:main',
        ],
    },
)
