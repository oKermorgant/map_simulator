from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value='batS')
    
    robot_description = sl.robot_description('turtlebot3_description', description_file='turtlebot3_waffle_pi.urdf', 
                                             description_dir='urdf')
    
    node_params = {}
    node_params['robot_description'] = robot_description
    #node_params['map'] = '/home/olivier/code/ros/src/ecn/mobro/ecn_navigation/maps/batS.yaml'
    node_params['max_height'] = 800.
    node_params['max_width'] = 1200.
    
    sl.node('simulation_2d', 'simulator_2d', parameters = node_params, output='screen')
    
    sl.node('robot_state_publisher', 'robot_state_publisher', parameters = {'robot_description': robot_description})
    
    
    return sl.launch_description()
