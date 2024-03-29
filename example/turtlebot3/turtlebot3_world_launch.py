from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('name', default_value='turtlebot1')
    sl.declare_arg('sim_wheels', default_value=False)

    # force joints to 0 if wheels are not simulated
    # this expression is not robust to lower case true or false
    zero_joints = sl.py_eval('not ', sl.arg('sim_wheels'))

    sl.include('map_simulator', 'simulation2d_launch.py',
               launch_arguments={'map': sl.find('turtlebot3_navigation2', 'map.yaml'),
                                 'map_server': True})

    name = sl.arg('name')

    with sl.group(ns=name):
        tf_prefix = sl.name_join(name, '/')
        sl.robot_state_publisher('map_simulator', 'turtlebot3_waffle_pi.urdf.xacro', xacro_args={'prefix': tf_prefix})

        # spawn in robot namespace to get robot_description
        sl.node('map_simulator', 'spawn',
                parameters = {'radius': 0.2,
                              'shape': 'circle',
                              'static_tf_odom': True,
                              'zero_joints': zero_joints,
                              'x': .5})

        with sl.group(if_arg='sim_wheels'):
            sl.node('map_simulator', 'kinematics.py')

        sl.node('slider_publisher', 'slider_publisher', arguments=[sl.find('map_simulator', 'cmd_vel.yaml')])

    sl.rviz(sl.find('map_simulator', 'turtlebot3.rviz'))

    return sl.launch_description()
