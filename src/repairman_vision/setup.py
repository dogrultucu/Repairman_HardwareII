from setuptools import setup

package_name = 'repairman_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/repairman_pipeline.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/repairman_demo.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dogru',
    maintainer_email='dogru@todo.todo',
    description='Repairman vision nodes (YOLO inference + test image publisher).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pub = repairman_vision.image_pub:main',
            'yolo_node = repairman_vision.yolo_node:main',
            'toolpath_node = repairman_vision.toolpath_node:main',
            'execute_node = repairman_vision.execute_node:main',
            'verify_node = repairman_vision.verify_node:main',
            'state_manager = repairman_vision.state_manager:main',
            'robot_description_pub = repairman_vision.robot_description_pub:main',
        ],
    },
)
