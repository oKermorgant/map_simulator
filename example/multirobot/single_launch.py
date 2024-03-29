from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('color', [255,0,0])
    sl.declare_arg('name', 'r2d2')
    sl.declare_arg('x', 0.)

    name = sl.arg('name')

    with sl.group(ns=name):

        # xacro RGB expects a string of 3 floats in [0-1]
        xacro_color = "'" + sl.py_eval("' '.join(str(c/255) for c in ", sl.arg('color'), ')') + "'"

        tf_prefix = name + '/'
        sl.robot_state_publisher('map_simulator', 'r2d2.xacro',
                                 xacro_args={'prefix': tf_prefix, 'rgb': xacro_color})

        # spawn in robot namespace to get robot_description
        robot_color = sl.py_eval("[c//2 for c in ", sl.arg('color'), ']')
        sl.node('map_simulator', 'spawn',
                parameters = {'radius': 0.4,
                              'shape': 'square',
                              'robot_color': robot_color,
                              'laser_color': sl.arg('color'),
                              'static_tf_odom': True,
                              'x': sl.arg('x')})

    return sl.launch_description()
