from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Путь к шаблонам Flask (если понадобится)
    web_led_share = get_package_share_directory('web_led')

    return LaunchDescription([
        # --- LED Node ---
        Node(
            package='led_control',
            executable='led_node',  # убедись, что setup.py правильный
            name='led_node',
            output='screen'
        ),

        # --- Web LED Node ---
        Node(
            package='web_led',
            executable='web_led_app',  # нужно в setup.py прописать entry_point для app.py
            name='web_led_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                # можно добавить параметры, например port, debug
            ],
            # если нужно передавать путь к шаблонам Flask:
            # extra_arguments=[{'templates_dir': os.path.join(web_led_share, 'templates')}]
        )
    ])
