from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('map', default_value=sl.find('map_simulator', 'house.yaml'))
    sl.declare_arg('max_height', default_value=800)
    sl.declare_arg('max_width', default_value=1200)
    sl.declare_arg('rate', default_value=20)
    sl.declare_arg('map_server', default_value=False)
    
    node_params = {}
    node_params['map'] = sl.arg('map')
    node_params['max_height'] = sl.arg('max_height')
    node_params['max_width'] = sl.arg('max_width')
    node_params['rate'] = sl.arg('rate')
    sl.node('map_simulator', 'simulator', parameters = node_params, output='screen')
    
    with sl.group(if_arg='map_server'):
        sl.node('nav2_map_server','map_server',name='map_server',
            parameters={'yaml_filename': sl.arg('map')})
        sl.node('nav2_lifecycle_manager','lifecycle_manager',name='lifecycle_manager_map',
        output='screen',
        parameters={'autostart': True, 'node_names': ['map_server']})
        
    return sl.launch_description()
