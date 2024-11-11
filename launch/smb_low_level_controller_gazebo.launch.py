from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare joint names as launch arguments
    left_wheel_joint_arg = DeclareLaunchArgument(
        'left_wheel_joint',
        default_value='left_wheel_joint',
        description='Name of the left wheel joint'
    )
    
    right_wheel_joint_arg = DeclareLaunchArgument(
        'right_wheel_joint',
        default_value='right_wheel_joint',
        description='Name of the right wheel joint'
    )

    return LaunchDescription([
        # Declare parameters for joints
        left_wheel_joint_arg,
        right_wheel_joint_arg,

        # Launch the differential drive controller node
        Node(
            package='differential_drive_controller_package',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen',
            parameters=[{
                'left_wheel_joint': LaunchConfiguration('left_wheel_joint'),
                'right_wheel_joint': LaunchConfiguration('right_wheel_joint')
            }]
        ),

        # Launch the ROS-Gazebo bridge node for left and right joint velocities
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='joint_velocity_bridge',
            output='screen',
            arguments=[
                '/left_joint_velocity_gazebo@std_msgs/msg/Float64@gz.msgs.Double',
                '/right_joint_velocity_gazebo@std_msgs/msg/Float64@gz.msgs.Double'
            ]
        ),
    ])
