#!/usr/bin/env python3
'''
    Internal simulation of steering-wheel robots

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import numpy as np

dt = 0.05

class Wheel:
    def __init__(self, joint):
        self.sign = joint.wheel_sign
    def move(self, val, w):
        return val + self.sign*w*dt

class SteeringJoint:
    def __init__(self, joint):
        self.low = joint.low
        self.up = joint.up
    def move(self, val, w):
        val += w*dt
        if self.low is not None:
            if val > self.up:
                return self.up
            elif val < self.low:
                return self.low
        return val

class Robot(Node):
    def __init__(self, joints, cmd_dim):
        super().__init__('kinematics')
        self.state = JointState()
        self.state.name = joints
        self.state.position = [0. for _ in range(len(joints))]
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 5)
        self.cmd = [0. for _ in range(cmd_dim)]

        self.cmd_vel = Twist()

        if self.declare_parameter('pub_cmd', False).value:
            self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)
        else:
            self.cmd_vel_pub = None

        if cmd_dim:
            self.cmd_sub = self.create_subscription(Float32MultiArray, 'cmd', self.cmd_callback, 5)

        self.timer = self.create_timer(dt, self.timer_callback)

    def timer_callback(self):
        self.update()
        self.publish()

    def cmd_callback(self, msg):
        self.cmd[:] = msg.data

    def publish(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(self.state)

        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.cmd_vel)

    def set_params(self, names):
        for name in names:
            self.declare_parameter(name, getattr(self, name))

class Unicycle(Robot):
    def __init__(self, names, left, right, b, r):
        super().__init__(names, 0)
        self.b = b
        self.r = r
        self.left = left
        self.right = right
        self.mimic = len(names) == 4

        self.set_params(['b','r'])

        # unicycle actually subscribes to cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 5)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def update(self):
        # forward latest cmd_vel to joint update
        v = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z

        for idx, joint, vel in ((0, self.left, (v-self.b*w)/self.r),
                                (1, self.right, (v+self.b*w)/self.r)):
            self.state.position[idx] = joint.move(self.state.position[idx], vel)
            if self.mimic:
                self.state.position[idx+2] = self.state.position[idx]

class Bicycle(Robot):
    def __init__(self, names, front, rear, beta, L, r):
        super().__init__(names, 2)
        self.L = L
        self.r = r
        self.front = front
        self.rear = rear
        self.beta = beta
        self.set_params(['L','r'])

    def update(self):
        # got (v, beta dot) in self.cmd
        v, bdot = self.cmd
        beta = self.state.position[2]

        # Twist
        self.cmd_vel.linear.x = v * np.cos(beta)
        self.cmd_vel.angular.z = v * np.sin(beta) / self.L

        # wheel velocities
        wf = v/self.r
        wr = self.cmd_vel.linear.x / self.r

        for idx, joint, vel in ((0, self.front, wf),
                                (1, self.rear, wr),
                                (2, self.beta, bdot)):
            self.state.position[idx] = joint.move(self.state.position[idx], vel)


class TwoSteering(Robot):
    def __init__(self, names, front, rear, beta1, beta2, L, r):
        super().__init__(names, 3)
        self.L = L
        self.r = r
        self.set_params(['L','r'])
        self.front = front
        self.rear = rear
        self.beta1 = beta1
        self.beta2 = beta2

    def update(self):
        # got (v, beta dot) in self.cmd
        v1, bdot1, bdot2 = self.cmd
        beta1,beta2 = self.state.position[2:4]
        c1,s1 = np.cos(beta1), np.sin(beta1)
        c2,s2 = np.cos(beta2), np.sin(beta2)
        v2 = v1*c1/c2

        # Twist
        self.cmd_vel.linear.x = v1 * c1
        self.cmd_vel.linear.y = v2 * s2
        self.cmd_vel.angular.z = (v1*s1-v2*s2) / self.L

        # wheel velocities
        wf = v1/self.r
        wr = v2/self.r

        for idx, joint, vel in ((0, self.front, wf),
                                (1, self.rear, wr),
                                (2, self.beta1, bdot1),
                                (3, self.beta2, bdot2)):
            self.state.position[idx] = joint.move(self.state.position[idx], vel)

def create_robot():
    # get model xml
    from rcl_interfaces.srv import GetParameters
    from urdf_parser_py.urdf import URDF
    node = Node('rsp_client')
    client = node.create_client(GetParameters, 'robot_state_publisher/get_parameters')
    client.wait_for_service()
    req = GetParameters.Request()
    req.names = ['robot_description']
    res = client.call_async(req)
    while rclpy.ok():
        rclpy.spin_once(node)
        if res.done():
            model = URDF.from_xml_string(res.result().values[0].string_value)
            break
    node.destroy_node()

    root = model.get_root()

    def sk(u):
        return np.matrix([[0,-u[2],u[1]],[u[2],0,-u[0]],[-u[1],u[0],0]])

    def Rot(theta,u):
        uuT = np.dot(np.reshape(u,(3,1)), np.reshape(u, (1,3)))
        return np.cos(theta)*np.eye(3) + np.sin(theta)*sk(u) + (1-np.cos(theta))*uuT

    def Homogeneous(joint):
        t = np.array(joint.origin.position).reshape(3,1)
        rpy = joint.origin.rotation
        R = Rot(rpy[2],[0,0,1])*Rot(rpy[1],[0,1,0])*Rot(rpy[0],[1,0,0])
        return np.matrix(np.vstack((np.hstack((R,t)), [0,0,0,1])))

    def to_list(t):
        return t.flatten().tolist()[0]

    class Joint:

        @staticmethod
        def identify(joints, prop):
            j1, j2 = joints
            if getattr(j2, prop) < getattr(j1, prop):
                return j1, j2
            return j2, j1

        def __init__(self, joint):

            self.name = joint.name
            self.low = self.up = None
            if joint.limit is not None:
                self.low = joint.limit.lower
                self.up = joint.limit.upper
            if self.low == self.up:
                self.low = self.up = None

            # get chain to root
            joints = [joint]
            while True:
                for prev in model.joints:
                    if prev.child == joints[-1].parent:
                        joints.append(prev)
                        break

                if joints[-1].parent == root:
                    break

            M = np.matrix(np.eye(4))
            for j in reversed(joints):
                M *= Homogeneous(j)

            # get position
            self.pos = to_list(M[:3,3])
            self.x, self.y, self.z = self.pos

            # get horizontal (wheel + sign) / vertical
            self.axis = to_list(M[:3,:3] * np.matrix(joint.axis).T)
            self.wheel_sign = min((-1,0,1), key = lambda y: abs(y-self.axis[1]))

    # parse model, guess role of each joint
    joints = [Joint(joint) for joint in model.joints if joint.type in ('continuous','revolute')]

    # identify wheels vs steering joints
    wheels = [joint for joint in joints if joint.wheel_sign]
    steering = [joint for joint in joints if not joint.wheel_sign]

    if len(wheels) not in (2,4) or len(steering) not in (0,1,2):
        msg = ['Cannot identify robot type from its joints']
        for j in joints:
            msg.append(f'  - {j.name} at {j.pos} with axis {j.axis}')
        raise(RuntimeError('\n'.join(msg)))

    # we assume the wheels have the same radius
    r = 0.5* (wheels[0].z + wheels[1].z)

    if len(steering) == 0:
        # unicycle
        if len(wheels) == 2:
            left, right = Joint.identify(wheels, 'y')
            names = [left.name, right.name]
        else:
            left = [w for w in wheels if w.y > 0]
            right = [w for w in wheels if w.y < 0]
            names = [left[0].name,right[0].name,left[1].name,right[1].name]
            left = left[0]
            right = right[0]
        return Unicycle(names,
                        Wheel(left),
                        Wheel(right),
                        left.y - right.y,
                        r)

    front, rear = Joint.identify(wheels, 'x')

    if len(steering) == 1:
        return Bicycle([front.name, rear.name, steering[0].name],
                       Wheel(front),
                       Wheel(rear),
                       SteeringJoint(steering[0]),
                       front.x-rear.x,
                       r)

    # two-steering
    beta1, beta2 = Joint.identify(steering, 'x')
    return TwoSteering([front.name, rear.name, beta1.name, beta2.name],
                       Wheel(front),
                       Wheel(rear),
                       SteeringJoint(beta1),
                       SteeringJoint(beta2),
                       front.x-rear.x,
                       r)


rclpy.init()

# guess one of the 3 robot types
robot = create_robot()

rclpy.spin(robot)

robot.destroy_node()
