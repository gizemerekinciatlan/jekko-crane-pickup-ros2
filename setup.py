from setuptools import setup
import glob
import os 

package_name = 'jekko'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/model/meshes', glob.glob('model/meshes/*.dae')),
        ('share/' + package_name + '/model', ['model/jekko_urdf.urdf']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
        ('share/' + package_name + '/config', ['config/config.rviz']),
],

    install_requires=[
    'setuptools',
    'opencv-python',
    'numpy',
    'filterpy', 
    'rclpy',
    'zlib',
    'pybase64',
    'pandas',
    'queue',
    'tf2_ros'
],

    zip_safe=True,
    maintainer='Görkem Can Ertemli & Gizem Erekinci Atlan',
    maintainer_email='goerkem.can.ertemli@rwth-aachen.de & gizem.erekinci.atlan@rwth-aachen.de',
    description='This is a package for moving Jekko XPS532 Crane to pick up a material by using aruco codes and camera system.',
    license='RWTH Aachen University',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jekko_code = jekko.jekko_code:main',
            "camera_talker = jekko.camera_getter:main",
        ],
    },
)
