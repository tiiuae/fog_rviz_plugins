import os
import launch
import argparse

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def search_and_replace(filename, search_key, replace_key):
    with open(filename, 'r', encoding='utf-8') as f:
        content = f.read()
        
        if search_key not in content:
            return filename

    replaced = content.replace(search_key, replace_key)

    tmp_filename = filename + '.tmp'
    with open(tmp_filename, 'w', encoding='utf-8') as f:
        f.write(replaced)
        print('Created a custom RVIZ2 config file %s' % tmp_filename)

    return tmp_filename

def generate_launch_description():
    
    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')
    
    ld = launch.LaunchDescription()
    
    pkg_name = "fog_rviz_plugins"

    rviz_config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'default.rviz')

    rviz_config_file = search_and_replace(rviz_config_path, 'REPLACE_ME', str(DRONE_DEVICE_ID))

    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen')
    )
    return ld
