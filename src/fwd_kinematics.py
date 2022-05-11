#!/usr/bin/env python
'''
    Internal simulation of steering-wheel robots

'''
import rospy
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
                
class Robot:
    def __init__(self, joints, cmd_dim):
        self.state = JointState()
        self.state.name = joints
        self.state.position = [0 for _ in range(len(joints))]
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
        self.cmd = [0 for _ in range(cmd_dim)]

        self.cmd_vel = Twist()        
        if rospy.get_param('~pub_cmd', False):
            self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        else:
            self.cmd_vel_pub = None
            
        if cmd_dim:
            self.cmd_sub = rospy.Subscriber('cmd', Float32MultiArray, self.cmd_callback)
            
    def cmd_callback(self, msg):
        self.cmd[:] = msg.data  
        
    def publish(self):
        self.state.header.stamp = rospy.Time.now()
        self.joint_pub.publish(self.state)
        
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(self.cmd_vel)
            
    def set_params(self, names):
        for name in names:
            rospy.set_param('~'+name, getattr(self, name))

class Unicycle(Robot):
    def __init__(self, names, left, right, b, r):
        super().__init__(names, 0)
        self.b = b
        self.r = r
        self.left = left
        self.right = right
        
        self.set_params(['b','r'])
        
        # unicycle actually subscribes to cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        
    def update(self):
        # forward latest cmd_vel to joint update
        v = self.cmd_vel.linear.x
        w = self.cmd_vel.angular.z
        
        for idx, joint, vel in ((0, self.left, (v+self.b*w)/self.r),
                                (1, self.right, (v-self.b*w)/self.r)):
            self.state.position[idx] = joint.move(self.state.position[idx], vel)        
        
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
        self.cmd_vel.angular.z = v * np.sin(beta) / self.L
        self.cmd_vel.linear.x = v * np.cos(beta)
        
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
        
        
        
                
    
def create_robot():
    from urdf_parser_py.urdf import URDF
    model = URDF.from_parameter_server()
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
            if np.allclose(self.axis, [0,1,0]):
                self.wheel_sign = 1
            elif np.allclose(self.axis, [0,-1,0]):
                self.wheel_sign = -1
            else:
                self.wheel_sign = 0

    # parse model, guess role of each joint
    joints = []

    for joint in model.joints:                
        if joint.type in ('continuous','revolute'):
            joints.append(Joint(joint))
            
    # identify wheels
    wheels = [joint for joint in joints if joint.wheel_sign]
    steering = [joint for joint in joints if not joint.wheel_sign]
    
    if len(wheels) != 2 or len(steering) not in (0,1,2):
        msg = ['Cannot identify robot type from its joints']
        for j in joints:
            msg.append(f'  - {j.name} at {j.pos} with axis {j.axis}')        
        raise(RuntimeError('\n'.join(msg)))
    
    r = 0.5* (wheels[0].z + wheels[1].z)
            
    if len(steering) == 0:
        # unicycle
        left, right = Joint.identify(wheels, 'y')
        return Unicycle([left.name, right.name], 
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
    return Bicycle([front.name, rear.name, beta1.name, beta2.name],
                       Wheel(front),
                       Wheel(rear),
                       SteeringJoint(beta1),
                       SteeringJoint(beta2),
                       front.x-rear.x,
                       r)

rospy.init_node('fwd_kinematics')

# guess one of the 3 robot types
robot = create_robot()


loop = rospy.Rate(1./dt)

while not rospy.is_shutdown():
    robot.update()
    robot.publish()
    loop.sleep()
