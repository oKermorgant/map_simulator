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

        sl.include('map_simulator', 'single_launch.py',
                   launch_arguments={'name': name, 'color': str(color), 'x': float(name[-1])/2})

    return sl.launch_description()
