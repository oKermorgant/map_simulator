from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()
    
    sl.declare_arg('name', default_value='waffle')
    
    # where the robot is spawned
    x = 1.
    
    name = sl.arg('name')
    
    with sl.group(ns=name):
        tf_prefix = sl.name_join(name, '/')
        sl.robot_state_publisher('map_simulator', 'turtlebot3_waffle_pi.urdf.xacro', xacro_args={'prefix': tf_prefix})
        
        # spawn in robot namespace to get robot_description
        sl.node('map_simulator', 'spawn', parameters = [{'zero_joints': True, 'static_tf_odom': True, 'radius': .2, 'x': x, 'robot_color': [50,50,50], 'laser_color': [0,255,0]}])
        
    
    return sl.launch_description()
