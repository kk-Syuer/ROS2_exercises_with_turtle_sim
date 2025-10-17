from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='minimal_param_node',
            name='custom_minimal_param_node',
            
            #Are used to redirect the output of the node to the console. 
            #The reason why this is needed is because nodes launched from a launch file are not directly attached to the std_output of your console.
            output='screen',
            emulate_tty=True,


            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])
