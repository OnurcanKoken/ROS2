from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    number_publisher_node = Node(
        package="my_py_pkg",
        executable="number_publisher",
        name="my_number_publisher", # rename the node
        remappings=[
            remap_number_topic  # rename topic
        ],
        parameters=[
            {"number_to_publish": 4},
            {"publish_frq": 5.0}
        ]
    )
    
    number_counter_node = Node(
        package="my_cpp_pkg_",
        executable="number_counter",
        name="my_number_counter", # rename the node
        remappings=[
            remap_number_topic,  # rename topic
            ("number_count", "my_number_count") # rename topic
        ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(number_counter_node)
    return ld