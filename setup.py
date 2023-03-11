from setuptools import setup

package_name = 'webots_ros2_conveyor'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', [
    'worlds/ure.wbt',
]))
data_files.append(('share/' + package_name + '/resource', [
    'resource/universal_robots_ure.urdf'
]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='2023.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools','launch'],
    zip_safe=True,
    maintainer='Matthew',
    maintainer_email='mwlock@kth.se',
    description='Conveyor belt ROS2 interface for Webots.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
