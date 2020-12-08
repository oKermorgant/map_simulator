from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher()
    
    sl.declare_arg('color', [255,0,0])
    sl.declare_arg('name', 'r2d2')
    
    name = sl.arg('name')
    
    with sl.group(ns=name):
        
        # build xacro RGB (from 0-255 to 0-1)
        xacro_color = sl.name_join("'", sl.py_eval("' '.join(str(c/255) for c in ", sl.arg('color'), ')'), "'")
        tf_prefix = sl.name_join(name, '/')
        sl.robot_state_publisher('simulation_2d', 'r2d2.xacro', xacro_args={'prefix': tf_prefix, 'rgb': xacro_color})
        
        # spawn in robot namespace to get robot_description
        # make robot darker than its laser
        robot_color = sl.py_eval("[c//2 for c in ", sl.arg('color'), ']')
        sl.node('simulation_2d', 'spawn', parameters = [{'shape': 'square'}, {'robot_color': robot_color}, {'laser_color': sl.arg('color')}])

    return sl.launch_description()
