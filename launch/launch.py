from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tether_tracker",
            executable="tether_tracker",
            output="screen"
        ),
        Node(
            package="synchronizer",
            executable="synchro_node",
            output="screen"
        ),

#         Node(
 #            package="lgs_bt",
 #            executable="behavior_tree",
  #           output="screen"
   #      ),

    ])
