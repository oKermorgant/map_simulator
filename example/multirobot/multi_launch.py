from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()

    # run the simulation
    sl.include('map_simulator', 'simulation2d_launch.py',
            launch_arguments={'map': sl.find('map_simulator', 'house.yaml'),
                              'map_server': True})

    # also run RViz
    sl.rviz(sl.find('map_simulator', 'r2d2.rviz'))

    # and spawn a slider controller
    sl.node('slider_publisher', arguments = [sl.find('map_simulator', 'cmd_vel_multi.yaml')])

    # spawn 3 robots
    for name, color in (('r2d2', [255,0,0]),
                        ('r2d3', [0,255,0]),
                        ('r2d4', [0,0,255])
                        ):

        with sl.group(ns=name):

            # xacro RGB expects a string of 3 floats in [0-1]
            xacro_color = f'"{' '.join(str(c/255) for c in color)}"'

            tf_prefix = name + '/'
            sl.robot_state_publisher('map_simulator', 'r2d2.xacro',
                                    xacro_args={'prefix': tf_prefix, 'rgb': xacro_color})

            # spawn in robot namespace to get robot_description
            sl.node('map_simulator', 'spawn',
                    parameters = {'radius': 0.4,
                                'shape': 'square',
                                'robot_color': color,
                                'laser_color': [c//2 for c in color],
                                'static_tf_odom': True,
                                'x': float(name[-1])/2})

    return sl.launch_description()
