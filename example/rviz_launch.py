from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.node('rviz2','rviz2', output='log', arguments=['-d', sl.find('map_simulator', 'r2d2_turtlebot.rviz')])

    return sl.launch_description()
