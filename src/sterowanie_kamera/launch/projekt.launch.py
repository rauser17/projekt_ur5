import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ścieżka do obrazka (zakładamy, że wrzucisz go do folderu pakietu)
    # Ale dla uproszczenia w launchu użyjemy ścieżki bezwzględnej lub argumentu
    # Najbezpieczniej dla zaliczenia: każemy userowi podać obrazek lub używamy domyślnego
    
    img_path = '~/ros2_ws/test_obraz.png' 
    # Upewnij się, że ten plik tam jest, lub zmień na swoją ścieżkę
    # Wygodniej: po prostu pobierzemy go przy uruchomieniu lub założymy że jest w katalogu domowym

    return LaunchDescription([
        # 1. Symulator kamery (Statyczny obrazek - to działa!)
        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='symulator_kamery',
            arguments=[os.path.expanduser('~/ros2_ws/test_obraz.png')], # Ścieżka do Twojego pliku
            remappings=[('/image_raw', '/burger/image')]
        ),

        # 2. Symulator Robota (Turtlesim)
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='symulator_zolwia'
        ),

        # 3. Twój węzeł sterujący
        Node(
            package='sterowanie_kamera',
            executable='sterownik',
            name='moj_interfejs',
            output='screen'
        ),
    ])
