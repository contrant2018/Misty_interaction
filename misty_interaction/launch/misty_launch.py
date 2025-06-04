from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable(name='DISABLE_NNPACK', value='1'),

        ExecuteProcess(
            cmd=['python3', '-m', 'misty_interaction.TTSNode'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', '-m', 'misty_interaction.SendToMisty'],
            output='screen'
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'run', 'misty_interaction', 'text_input_node'],
        #     output = 'screen'
        #     ),
    ])
