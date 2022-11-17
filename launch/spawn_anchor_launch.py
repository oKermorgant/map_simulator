from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()

    sl.declare_arg('frame','anchor',description='Name of the anchor frame')
    sl.declare_arg('x',0.,description='x-position of the anchor')
    sl.declare_arg('y',0.,description='y-position of the anchor')
    sl.declare_arg('range_min',0.,description='Min range')
    sl.declare_arg('range_max',100.,description='Max range')
    sl.declare_arg('covariance',0.01,description='covariance factor of the anchor range')
    sl.declare_arg('covariance_real',0.01,description='real covariance factor of the anchor range')
    sl.declare_arg('publish_gt',True,description='Publish real anchor position')

    parameters = sl.arg_map(('frame','x','y','range_min','range_max','covariance','covariance_real'))
    # ensure floating points in case integers are passed
    for name in ('x','y','range_min','range_max','covariance','covariance_real'):
        parameters[name] = sl.py_eval('float(', sl.arg(name), ')')

    sl.node('map_simulator','add_anchor', parameters = parameters)

    with sl.group(if_arg='publish_gt',ns='/anchors'):
        sl.node('tf2_ros','static_transform_publisher',name=sl.name_join(sl.arg('frame'), '_tf'),
            arguments = [sl.arg('x'),sl.arg('y'),'0','0','0','0','map',sl.arg('frame')])

    return sl.launch_description()
