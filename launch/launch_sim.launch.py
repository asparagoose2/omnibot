import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='omnibot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
                # PythonLaunchDescriptionSource([os.path.join(
                    # get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            #  )
    
    # setup world file
    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'basic_4_turns.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world}.items()

    )    

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'omnibot'],
                        output='screen')
    
    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('omnibot'), 'config', 'omnibot.config.rviz')]
        )



    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            name='world',
            default_value=[os.path.join(get_package_share_directory(package_name), 'worlds', 'final_v5.world'), ''],
            description='SDF world file'
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen'
        ),
        rsp,
        spawn_entity,
        rviz
    ])