from simple_launch import SimpleLauncher
from map_simulator.srv import AddAnchor


def generate_launch_description():
    sl = SimpleLauncher()

    anchor = AddAnchor.Request()

    sl.declare_arg('reference_frame','map',description='Name of the world or map frame')
    sl.declare_arg('publish_gt',True,description='Publish real anchor position')

    fields = list(anchor.get_fields_and_field_types().keys())
    for field in fields:
        sl.declare_arg(field, getattr(anchor, field))

    sl.call_service('/simulator/add_anchor', request = sl.arg_map(fields))

    with sl.group(if_arg='publish_gt',ns='/anchors'):
        sl.node('tf2_ros','static_transform_publisher',name=sl.arg('frame_id')+'_tf',
            arguments = ['--x',sl.arg('x'), '--y', sl.arg('y'), '--frame-id', 'map','--child-frame-id', sl.arg('frame_id')])

    return sl.launch_description()
