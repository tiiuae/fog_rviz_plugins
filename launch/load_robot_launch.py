from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

import sys
import random

def color_to_text(color):
    return "{:.0e} {:.0e} {:.0e} {:.0e}".format(color[0], color[1], color[2], color[3])

def get_uav_color(uav_name):
    transp = 0.2
    colors = dict(
        uav1=(1.0, 0.0, 0.0, transp),
        uav2=(0.0, 1.0, 0.0, transp),
        uav3=(0.0, 0.0, 1.0, transp),
#
        uav4=(1.0, 1.0, 0.0, transp),
        uav5=(1.0, 0.0, 1.0, transp),
        uav6=(0.0, 1.0, 1.0, transp),
#
        uav7=(1.0, 0.5, 0.5, transp),
        uav8=(0.5, 1.0, 0.5, transp),
        uav9=(0.5, 0.5, 1.0, transp),

        uav=(0.6, 0.6, 0.6, transp)
    )
    if uav_name in colors:
        return color_to_text(colors[uav_name])
    else:
        return color_to_text(random.choice(list(colors.values())))

def load_model(fname, uav_name, pkg_path):
    model = ""
    uav_color = get_uav_color(uav_name)

    with open(fname, 'r') as fhandle:
        for line in fhandle.readlines():
            line = line.replace("[REPLACEME]uav_color[/REPLACEME]", uav_color)
            line = line.replace("[REPLACEME]uav_name[/REPLACEME]", uav_name)
            line = line.replace("[REPLACEME]path[/REPLACEME]", pkg_path)
            model = model + line
    return model


def generate_launch_description():

    ld = LaunchDescription()

    # environment variables
    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    #namespace declarations
    namespace = DRONE_DEVICE_ID

    # frame names
    base_frame = DRONE_DEVICE_ID + "/fcu"
    uav_marker_frame = base_frame + "/uav_marker"
    props_frame = base_frame + "/props"
    arms_frame = base_frame + "/arms"
    arms_red_frame = base_frame + "/arms_red"

    # URDF/xacro file to be loaded by the Robot State Publisher node
    this_package_path = get_package_share_directory('fog_rviz_plugins')
    xml_path = os.path.join(
        this_package_path,
        'data', 'x500.xml'
    )
    robot_desc = load_model(xml_path, base_frame, this_package_path)
    
    # node definitions
    ld.add_action(
        Node(
            namespace = namespace,
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc}
            ],
        ),
    )

    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "tf_published_uav_marker_link",
            arguments = ["0", "0", "0", "0", "0", "0", base_frame, uav_marker_frame],
            output='screen',
        ),
    )
    
    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "tf_published_props_link",
            arguments = ["0", "0", "0", "0", "0", "0", base_frame, props_frame],
            output='screen',
        ),
    )

    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "tf_published_arms_link",
            arguments = ["0", "0", "0", "0", "0", "0", base_frame, arms_frame],
            output='screen',
        ),
    )

    ld.add_action(
        Node(
            namespace = namespace,
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            name= "tf_published_arms_red_link",
            arguments = ["0", "0", "0", "0", "0", "0", base_frame, arms_red_frame],
            output='screen',
        ),
    )


    return ld
