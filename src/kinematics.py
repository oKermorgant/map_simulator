#!/usr/bin/env python3
'''
    Internal simulation of steering-wheel robots

'''
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
from math import isnan, cos, sin, tan, atan, sqrt
from sys import exit

try:
    from ackermann_msgs.msg import AckermannDrive
except ModuleNotFoundError:
    AckermannDrive = None
try:
    from four_wheel_steering_msgs.msg import FourWheelSteering
except ModuleNotFoundError:
    FourWheelSteering = None


def sign(val):
    return -1 if val < 0 else 1


class Wheel:
    dt = 0.2

    def __init__(self, joint):
        self.name = joint.name
        self.sign = joint.wheel_sign
        self.val = 0.

    def move(self, w):
        self.val += self.sign*w*Wheel.dt


class SteeringJoint:
    use_angle_cmd = True

    def __init__(self, joint=None, low=None, up=None):
        if joint is not None:
            self.name = joint.name
            self.low = joint.low
            self.up = joint.up
        else:
            self.name = 'steering'
            self.low = low
            self.up = up
        self.val = 0.

    def adapt(self, angle, w):
        '''
        Returns current angle and velocity from the desired one
        If desired angle is NaN, accepts any velocity
        Otherwise, w is the max change rate, adapt it to the actual one
        '''
        if SteeringJoint.use_angle_cmd and not isnan(angle):
            if w == 0. or isnan(w):
                w = (angle - self.val)/Wheel.dt
            else:
                # w is a max rate
                rate = (angle - self.val)/Wheel.dt
                if abs(rate) < abs(w):
                    w = rate
                else:
                    w = sign(rate)*w
        return self.val, w

    def move(self, w):
        self.val += w*Wheel.dt
        if self.low is not None:
            if self.val < self.low:
                self.val = self.low
            elif self.val > self.up:
                self.val = self.up


class Robot(Node):
    def __init__(self, joints, cmd_type = None):

        super().__init__('kinematics')
        self.joints = joints
        self.state = JointState()
        self.state.name = [joint.name for joint in joints]
        self.state.position = [0. for _ in range(len(joints))]
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 5)

        from rcl_interfaces.msg import ParameterDescriptor
        dt_description = ParameterDescriptor(description = 'Sampling time [s]')
        Wheel.dt = self.declare_parameter('dt', 0.1, descriptor=dt_description).value

        angle_description = ParameterDescriptor(description = 'Whether to use the steering angle cmd or only the velocity')
        SteeringJoint.use_angle_cmd = self.declare_parameter('use_angle_cmd', True, descriptor=angle_description).value

        self.cmd_vel = Twist()
        self.cmd_vel_pub = None

        if cmd_type is not None:
            if self.declare_parameter('pub_cmd', True).value:
                self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 5)
            self.cmd = cmd_type()
            self.cmd_sub = self.create_subscription(cmd_type, 'cmd', self.cmd_callback, 5)

        self.timer = self.create_timer(Wheel.dt, self.timer_callback)

    def timer_callback(self):
        self.update()
        self.publish()

    def cmd_callback(self, msg):
        self.cmd = msg

    def publish(self):
        self.state.header.stamp = self.get_clock().now().to_msg()
        self.state.position = [joint.val for joint in self.joints]
        self.joint_pub.publish(self.state)

        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.cmd_vel)

    def set_params(self, names):
        for name in names:
            self.declare_parameter(name, getattr(self, name))


class Unicycle(Robot):
    def __init__(self, left, right, b, r, left2=None, right2=None):
        super().__init__([j for j in (left,right,left2,right2) if j is not None], 0)
        self.b = b
        self.r = r
        self.left = left
        self.right = right
        self.mimic = left2 is not None
        self.cmd = Twist()

        self.set_params(['b','r'])

        # unicycle actually subscribes to cmd_vel and just rotates the wheels accordingly
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 5)

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def update(self):
        # forward latest cmd_vel to joint update
        v = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z

        self.left.move((v-self.b*w)/self.r)
        self.right.move((v+self.b*w)/self.r)

        if self.mimic:
            self.joints[2].val = self.left.val
            self.joints[3].val = self.right.val


class Bicycle(Robot):
    def __init__(self, front, rear, beta, L, r):
        super().__init__([front, rear, beta], AckermannDrive)
        self.L = L
        self.r = r
        self.set_params(['L','r'])

    def update(self):
        # got (v, beta dot) in self.cmd as AckermannDrive msg
        v = self.cmd.speed

        beta, bdot = self.joints[2].adapt(self.cmd.steering_angle, self.cmd.steering_angle_velocity)

        # corresponding Twist
        self.cmd_vel.linear.x = v * cos(beta)
        self.cmd_vel.angular.z = v * sin(beta) / self.L

        # wheel velocities
        wf = v/self.r
        wr = self.cmd_vel.linear.x / self.r

        for idx, w in enumerate((wf, wr, bdot)):
            self.joints[idx].move(w)


class Ackermann(Robot):
    def __init__(self, fls, frs, fl, fr, rl, rr, L, r, B):
        super().__init__([fls,frs,fl,fr,rl,rr,SteeringJoint(low=fls.low, up=fls.up)], AckermannDrive)
        self.L = L
        self.r = r
        self.B = B

        self.set_params(['L','r','B'])

    def update(self):
        v = self.cmd.speed
        beta, bdot = self.joints[-1].adapt(self.cmd.steering_angle, self.cmd.steering_angle_velocity)

        # Twist
        self.cmd_vel.linear.x = v * cos(beta)
        self.cmd_vel.angular.z = v * sin(beta) / self.L

        # update beta
        self.joints[-1].move(bdot)

        # update steering angles
        tb = tan(self.joints[-1].val)
        self.joints[0].val = atan(self.L*tb/(self.L+0.5*self.B*tb))
        self.joints[1].val = atan(self.L*tb/(self.L-0.5*self.B*tb))

        # update wheels, find curvature radius
        # velocity of imaginary centered rear wheel
        w = self.cmd_vel.linear.x/self.r

        # wheel velocities
        if abs(tb) < 1e-4:
            for i in range(2, 6):
                self.joints[i].move(w)
            return

        rho = self.L/tb
        ratio = w/rho

        rrl = rho-self.B/2
        rrr = rho+self.B/2
        rfl = sqrt(rrl**2+self.L**2)*sign(rho)
        rfr = sqrt(rrr**2+self.L**2)*sign(rho)

        for idx, dist in enumerate((rfl, rfr, rrl, rrr)):
            self.joints[idx+2].move(dist*ratio)


class TwoSteering(Robot):
    def __init__(self, front, rear, beta1, beta2, L, r):
        super().__init__([front, rear, beta1, beta2], FourWheelSteering)
        self.L = L
        self.r = r
        self.set_params(['L','r'])

    def update(self):
        # got (v1, beta1 dot, beta2 dot) in self.cmd as FourWheelSteering
        v1 = self.cmd.speed
        beta1, bdot1 = self.joints[2].adapt(self.cmd.front_steering_angle,
                                            self.cmd.front_steering_angle_velocity)
        beta2, bdot2 = self.joints[3].adapt(self.cmd.rear_steering_angle,
                                            self.cmd.rear_steering_angle_velocity)

        c1,s1 = cos(beta1), sin(beta1)
        c2,s2 = cos(beta2), sin(beta2)
        v2 = v1*c1/c2

        # Twist
        self.cmd_vel.linear.x = v1 * c1
        self.cmd_vel.linear.y = v2 * s2
        self.cmd_vel.angular.z = (v1*s1-v2*s2) / self.L

        # wheel velocities
        wf = v1/self.r
        wr = v2/self.r

        for idx, w in enumerate((wf, wr, bdot1, bdot2)):
            self.joints[idx].move(w)


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
        return cos(theta)*np.eye(3) + sin(theta)*sk(u) + (1-cos(theta))*uuT

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

    if len(joints) == 0:
        exit(0)

    # identify wheels vs steering joints
    wheels = [joint for joint in joints if joint.wheel_sign]
    steering = [joint for joint in joints if not joint.wheel_sign]

    if len(wheels) not in (2,4) or len(steering) not in (0,1,2):
        msg = ['Cannot identify robot type from its joints']
        for j in joints:
            msg.append(f'  - {j.name} at {j.pos} with axis {j.axis}')
        raise(RuntimeError('\n'.join(msg)))

    # we assume the wheels have the same radius
    r = 0.5*(wheels[0].z + wheels[1].z)

    if len(steering) == 0:
        # unicycle
        if len(wheels) == 2:
            left, right = Joint.identify(wheels, 'y')
            left2 = right2 = None
        else:
            left = [w for w in wheels if w.y > 0]
            right = [w for w in wheels if w.y < 0]
            left, left2 = left
            right, right2 = right
        return Unicycle(Wheel(left),
                        Wheel(right),
                        left.y - right.y, r,
                        left2, right2)

    if len(wheels) == 2:
        # basic model
        front, rear = Joint.identify(wheels, 'x')

        if len(steering) == 1:
            return Bicycle(Wheel(front),
                        Wheel(rear),
                        SteeringJoint(steering[0]),
                        front.x-rear.x,
                        r)

        # two-steering
        beta1, beta2 = Joint.identify(steering, 'x')
        return TwoSteering(Wheel(front),
                        Wheel(rear),
                        SteeringJoint(beta1),
                        SteeringJoint(beta2),
                        front.x-rear.x,
                        r)

    # Probably Ackermann (2 steering / 4 wheels)
    fls, frs = Joint.identify(steering, 'y')
    fl,fr,rl,rr = sorted(wheels, key = lambda w: -w.x - 0.001*w.y)
    return Ackermann(SteeringJoint(fls),SteeringJoint(frs),
                     Wheel(fl),Wheel(fr),Wheel(rl),Wheel(rr), fl.x-rl.x, r, rl.y - rr.y)


rclpy.init()

# guess one of the 3 robot types
robot = create_robot()

rclpy.spin(robot)

robot.destroy_node()
