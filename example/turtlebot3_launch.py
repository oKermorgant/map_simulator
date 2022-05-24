from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()
    
    sl.declare_arg('name', default_value='turtlebot1')
    sl.declare_arg('sim_wheels', default_value=False)
    
    # force joints to 0 if wheels are not simulated
    zero_joints = sl.py_eval('not ', sl.arg('sim_wheels'))
        
    
    # where the robot is spawned
    x = 1.
    
    name = sl.arg('name')
    
    with sl.group(ns=name):
        tf_prefix = sl.name_join(name, '/')
        sl.robot_state_publisher('map_simulator', 'turtlebot3_waffle_pi.urdf.xacro', xacro_args={'prefix': tf_prefix})
        
        # spawn in robot namespace to get robot_description
        
        
        sl.node('map_simulator', 'spawn', parameters = {'zero_joints': zero_joints, 'static_tf_odom': True, 'radius': .2, 'x': x, 'robot_color': [50,50,50], 'laser_color': [0,255,0]})
        
        with sl.group(if_arg='sim_wheels'):
            sl.node('map_simulator', 'kinematics.py')
        
        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('map_simulator', 'cmd_vel.yaml')])
        
    
    return sl.launch_description()
