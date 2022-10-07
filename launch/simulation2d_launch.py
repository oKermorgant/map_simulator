from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value=sl.find('map_simulator', 'house.yaml'))
    sl.declare_arg('max_height', default_value=800)
    sl.declare_arg('max_width', default_value=1200)
    sl.declare_arg('rate', default_value=20)
    sl.declare_arg('map_server', default_value=False)
    sl.declare_arg('display', default_value=True)
    
    sl.node('map_simulator', 'simulator', 
            parameters = sl.arg_map('map', 'max_height', 'max_width', 'rate', 'display'),
            output='screen')
    
    with sl.group(if_arg='map_server'):
        sl.node('nav2_map_server','map_server',name='map_server',
            parameters={'yaml_filename': sl.arg('map')})
        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager_map',
        output='screen',
        parameters={'autostart': True, 'node_names': ['map_server']})
        
    return sl.launch_description()
