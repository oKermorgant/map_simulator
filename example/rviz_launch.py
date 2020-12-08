from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher()

    sl.node('rviz2','rviz2', output='log', arguments=['-d', sl.find('simulation_2d', 'r2d2_turtlebot.rviz')])
    
    # also assume perfect localization for R2D2 / waffle
    sl.node('tf2_ros', 'static_transform_publisher', name='r2d2_loc', arguments=['0','0','0','0','0','0','map','r2d2/odom'])
    sl.node('tf2_ros', 'static_transform_publisher', name='waffle_loc', arguments=['0','0','0','0','0','0','map','waffle/odom'])
    
    return sl.launch_description()
