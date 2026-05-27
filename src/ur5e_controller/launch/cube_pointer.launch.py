from launch import LaunchDescription
from launch_ros.actions import Node

CYCLONE = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}


def generate_launch_description():
    return LaunchDescription([

        # USB camera on /dev/video4
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            additional_env=CYCLONE,
            parameters=[{
                'video_device':  '/dev/video0',
                'image_width':   640,
                'image_height':  480,
                'pixel_format':  'yuyv',
                'framerate':     20.0,
            }],
        ),

        # Color detector — output='log' keeps detection spam out of the console
        Node(
            package='color_square_detector',
            executable='detector_node',
            name='color_square_detector',
            output='log',
            additional_env=CYCLONE,
            parameters=[
                '/home/laptop/UR/color_params.yaml',
                {'params_file': '/home/laptop/UR/color_params.yaml'},
            ],
        ),

        # Arm controller
        Node(
            package='ur5e_controller',
            executable='cube_pointer',
            name='cube_pointer',
            output='screen',
            additional_env=CYCLONE,
            parameters=[{
                # TCP pose at home/overview [x, y, z, rx, ry, rz]
                # From ROS pose: position + quaternion→rotvec conversion
                'home_pose': [-0.28297, -0.47154, 0.64359, -1.207, -2.896, -0.010],

                'motion_duration':   6.0,
                'approach_height':   0.10,

                # table_z = tcp_z - camera_height = 0.6436 - 0.660 = -0.016 m
                'table_z':          -0.016,
                'cube_height':       0.05,

                # Camera rotation offset relative to tool frame (tune if XY is wrong)
                'camera_yaw_deg':    180.0,

                # Rotate camera position ±45° around base Z for search sweeps
                'search_angles_deg': [45.0, -45.0],
            }],
        ),
    ])
