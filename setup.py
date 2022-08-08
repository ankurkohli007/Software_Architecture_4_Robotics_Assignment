from setuptools import setup

package_name = 'sofar_assignment'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/'+package_name]))
data_files.append(('share/'+package_name+'/launch', [
    'launch/robot_launch.py',
    'launch/navigation_launch.py',
    'launch/localization_launch.py',
    'launch/slam_launch.py'
     ]))
data_files.append(('share/'+package_name+'/params',[
    'params/keepout_params.yaml',
    'params/nav2_params.yaml',
    'params/keepout_params_1.yaml',
    'params/nav2_params_1.yaml',
    'params/keepout_params_2.yaml',
    'params/nav2_params_2.yaml',
    'params/keepout_params_3.yaml',
    'params/nav2_params_3.yaml',
    'params/emaro_keepout_params.yaml',
    'params/emaro_keepout_params_2.yaml',
    'params/emaro_nav2_params.yaml'
     ]))
data_files.append(('share/'+package_name+'/worlds', [
    'worlds/default.wbt',
    'worlds/emaro_lab.wbt'
    ]))
data_files.append(('share/'+package_name+'/resource', [
    'resource/default.rviz',
    'resource/ros2_control.yml',
     ]))
data_files.append(('share/'+package_name+'/URDF',[
    'URDF/tiago_webots.urdf'
    ]))
data_files.append(('share/'+package_name+'/maps',[
    'maps/keepout_mask.pgm',
    'maps/keepout_mask1.pgm',
    'maps/keepout_mask2.pgm',
    'maps/keepout_mask3.pgm',
    'maps/keepout_mask.yaml',
    'maps/keepout_mask1.yaml',
    'maps/keepout_mask2.yaml',
    'maps/keepout_mask3.yaml',
    'maps/map.pgm',
    'maps/map2.pgm',
    'maps/map.yaml',
    'maps/map2.yaml',
    'maps/emaro_map.yaml',
    'maps/emaro_map.pgm',
    'maps/emaro_keepout_mask.yaml',
    'maps/emaro_keepout_mask.pgm',
    'maps/emaro_keepout_mask2.yaml',
    'maps/emaro_keepout_mask2.pgm'
    ]))
data_files.append(('share/'+package_name+'/rviz',[
    'rviz/nav2_default_view.rviz'
    ]))
data_files.append(('share/'+package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salvatore',
    maintainer_email='salvo.dippolito@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'move_to_goal_exe = sofar_assignment.nav_to_pose:main'
        ],
    },
)
