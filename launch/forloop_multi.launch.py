import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description(): 


    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(get_package_share_directory("my_robot_description"),"urdf","robot_description.urdf.xacro"),
        description = "Absolute path to robot URDF file"
    )
    model2_arg = DeclareLaunchArgument(
        name = "model2",
        default_value = os.path.join(get_package_share_directory("my_robot_description"),"urdf","robot_description2.urdf.xacro"),
        description = "Absolute path to robot URDF file"
    )

    # Define arguments for each robot
    robot1_arg = DeclareLaunchArgument(
        name="robot1_x_spawn",
        default_value="0",
        description="X spawn position for robot 1"
    )
    robot2_arg = DeclareLaunchArgument(
        name="robot2_x_spawn",
        default_value="0.75",
        description="X spawn position for robot 2"
    )

    # Define parameters for robot description
    robot_description1 = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type=str)
    robot_description2 = ParameterValue(Command(["xacro ", LaunchConfiguration("model2")]), value_type=str)

    # Create nodes for each robot
    robot_state_publisher1 = Node(
         package= "robot_state_publisher",
         executable = "robot_state_publisher",
         parameters = [{"robot_description": robot_description1 }],
         namespace='robot1',
         remappings=[('/tf', '/robot1/tf'), ('/robot_description', '/robot1/robot_description')]



    )

    robot_state_publisher2 = Node(
         package= "robot_state_publisher",
         executable = "robot_state_publisher",
         parameters = [{"robot_description": robot_description2 }],
         namespace='robot2',  
         remappings=[('/tf', '/robot2/tf'), ('/robot_description', '/robot2/robot_description')]

    )

    # Set environment variable for Gazebo model path
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH" , os.path.join(get_package_prefix("my_robot_description"),"share"))

    # Include Gazebo launch files
    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch","gzserver.launch.py")
    ))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
         
        os.path.join(get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py")
    ))

    # Spawn robots using launch arguments
    spawn_robot1 = Node(
         package="gazebo_ros",
         executable="spawn_entity.py",
         arguments=["-entity", "robot1","-topic", "robot_description"],
         output="screen",
         parameters=[{"robot_description": robot_description1, "robot_name": "robot1", "x_spawn": LaunchConfiguration("robot1_x_spawn")}], 
         namespace='robot1'  
    )

    spawn_robot2 = Node(
         package="gazebo_ros",
         executable="spawn_entity.py",
         arguments=["-entity", "robot2","-topic", "robot_description"],
         output="screen",
         parameters=[{"robot_description": robot_description2, "robot_name": "robot2", "x_spawn": LaunchConfiguration("robot2_x_spawn")}], 
         namespace='robot2'  
    )

    # RViz node for visualization
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output= "screen"
    )

    return LaunchDescription([ model_arg,
                              model2_arg,
                              robot1_arg,
                              robot2_arg,
                              robot_state_publisher1,
                              robot_state_publisher2,
                              env_var,
                              start_gazebo_server,
                              start_gazebo_client,
                              spawn_robot1,
                              spawn_robot2,
                              rviz2_node                         
                             ])


#each robot have the same topic in rviz but 2 in gazebo #
