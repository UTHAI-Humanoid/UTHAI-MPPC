#!/usr/bin/env python
import rospy
import actionlib

from dynamixel_controllers.srv import SetSpeed
from std_msgs.msg import Float64, Header, Bool, Empty
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryFeedback
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from dynamixel_controllers.srv import TorqueEnable

class Uthai(object):
    def __init__(self):
        rospy.loginfo("Initial uthai_bringup")

        self.joint_names = ['r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee_pitch', 'r_ankle_pitch',
                            'r_ankle_roll', 'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee_pitch', 'l_ankle_pitch', 'l_ankle_roll']
        self.joint_names_rviz = ['r_hip_yaw_joint', 'r_hip_roll_joint', 'r_hip_pitch_joint', 'r_knee_pitch_joint', 'r_ankle_pitch_joint',
                                 'r_ankle_roll_joint', 'l_hip_yaw_joint', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_pitch_joint', 'l_ankle_pitch_joint', 'l_ankle_roll_joint']

        self.joint_state_sub = rospy.Subscriber(
            'uthai_controller/state', FollowJointTrajectoryFeedback, self.joint_state_callback)
        self.joint_state_pub = rospy.Publisher(
            '/uthai/joint_states', JointState, queue_size=10)
        self.joint_state_pub_rviz = rospy.Publisher(
            '/uthai/joint_states_rviz', JointState, queue_size=10)

        self.joint_state = JointState()
        self.joint_command_sub = rospy.Subscriber('/uthai/joint_command',JointTrajectory, self.joint_command_callback)

        self.torque_enable = rospy.Subscriber('/uthai/torque_enable', Bool, self.torque_enable_callback)
        # self.get_position = rospy.Subscriber('/uthai/get_position', Empty, self.get_position_callback)
        ## Action client ##
        self.jta = actionlib.SimpleActionClient(
            '/uthai_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        rospy.loginfo("Waiting fot joint trajectory action")
        self.jta.wait_for_server()
        rospy.loginfo("Found joint trajectory action!")

      
        self.torque_enable_services = [
            rospy.ServiceProxy('r_hip_yaw_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('r_hip_roll_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('r_hip_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('r_knee_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('r_ankle_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('r_ankle_roll_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_hip_yaw_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_hip_roll_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_hip_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_knee_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_ankle_pitch_controller/torque_enable', TorqueEnable),
            rospy.ServiceProxy('l_ankle_roll_controller/torque_enable', TorqueEnable)
 
            # rospy.ServiceProxy('r_knee_pitch_controller/torque_enable', TorqueEnable),
            # rospy.ServiceProxy('l_knee_pitch_controller/torque_enable', TorqueEnable),
        ]

    
    # def get_position_callback(self, msg):
    #     rospy.loginfo(self.joint_state.position)

    def torque_enable_callback(self, msg):
        rospy.loginfo(msg.data)
        for torque_enable_service in self.torque_enable_services:
            torque_enable_service(msg.data)




    def joint_command_callback(self, msg):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        goal.trajectory.points = msg.points
        rospy.loginfo("Joint move {}".format(msg.points))
        self.jta.send_goal_and_wait(goal)
        # rospy.loginfo(goal)


    def joint_state_callback(self, msg):
        self.joint_state.header = msg.header
        self.joint_state.name = msg.joint_names
        self.joint_state.position = msg.actual.positions
        self.joint_state.velocity = msg.actual.velocities
        self.joint_state_pub.publish(self.joint_state)

        self.joint_state.name = self.joint_names_rviz
        self.joint_state_pub_rviz.publish(self.joint_state)

    def joints_move(self, angles, duration):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
        rospy.loginfo("Joint move {}".format(angles))
        # rospy.loginfo(goal)

    # def joint_move(self,joint_name,angle,duration):
    #     g


def main():
    rospy.init_node("uthai_bringup")
    uthai = Uthai()
    # uthai.joint_command_enable()
    # uthai.joints_move([0.0]*12, 1.0)
    uthai.joints_move([0, -0.11, 0, 0, 0, 0, 0, 0.11, 0, 0, 0, 0],0.5)

    # rospy.sleep(3)

    uthai.joints_move( [0, -0.11, -0.15, 0.2, -0.1, 0, 0, 0.11, -0.15, 0.2, -0.1, 0], 1.0)
    rospy.sleep(5)

        # uthai.joints_move( [0, -0.11, -0.1, 0.2, -0.12, -0.2, 0, 0.11, -0.4, 0.7, -0.4, -0.2], 1.5)
        # uthai.joints_move( [0, -0.11, -0.4, 0.7, -0.4, 0.2, 0, 0.11, -0.1, 0.2, -0.12, 0.2], 1.5)
        
        
    #     #left step
    # uthai.joints_move( [0, -0.11, -0.25, 0.5, -0.35, -0.3, 0, 0.11, -0.1, 0.2, -0.15, -0.3], 0.8)
    # uthai.joints_move( [0, -0.11, 0.0, 0.2, -0.15, -0.25, 0, 0.2, -0.5, 1.0, -0.5, -0.25], 0.5)
    # uthai.joints_move( [0, -0.11, 0.1, 0.1, -0.2, 0.05, 0, 0.25, -0.25, 0.3, -0.1, 0.05], 0.15)

        #left step 2
    uthai.joints_move( [0, -0.11, -0.25, 0.5, -0.35, -0.3, 0, 0.11, -0.1, 0.2, -0.15, -0.3], 0.8)
    uthai.joints_move( [0, -0.11, 0.0, 0.2, -0.15, -0.25, 0, 0.2, -0.5, 1.0, -0.5, -0.25], 0.4)
    uthai.joints_move( [0, -0.11, 0.1, 0.1, -0.2, 0.05, 0, 0.25, -0.25, 0.3, -0.1, 0.05], 0.2)

    for i in range(1):
        #                   RHY RHR     RHP     RKP     RAP     RAR     LHY LHR     LHP     LKP     LAP     LAR     TIME  
        # uthai.joints_move( [0,  -0.11,  -0.25,  0.4,    -0.35,  -0.3,   0,  0.12,   -0.1,   0.2,    -0.15,  -0.3],  0.8)
        # uthai.joints_move( [0,  -0.13,  0.0,    0.2,    -0.15,  -0.25,  0,  0.2,   -0.4,   1.0,    -0.5,   -0.25], 0.6)
        # uthai.joints_move( [0,  -0.2,   0.1,    0.1,    -0.2,   0.05,   0,  0.2,    -0.15,  0.3,    -0.1,   0.05],  0.15)
        # uthai.joints_move( [0, -0.11, -0.25, 0.5, -0.35, -0.3, 0, 0.11, -0.1, 0.2, -0.15, -0.3], 0.8)
        # uthai.joints_move( [0, -0.11, 0.0, 0.2, -0.15, -0.25, 0, 0.2, -0.5, 1.0, -0.5, -0.25], 0.4)
        # uthai.joints_move( [0, -0.11, 0.1, 0.1, -0.2, 0.05, 0, 0.25, -0.25, 0.3, -0.1, 0.05], 0.2)
        # uthai.joints_move( [0,  -0.11, -0.25, 0.5, -0.35, -0.3, 0, 0.11, -0.1, 0.2, -0.15, -0.3], 0.8)
        # uthai.joints_move( [0, -0.2,    0.0, 0.2, -0.15, 0.1, 0, 0.25, -0.5, 1.0, -0.5, -0.25], 0.4)
        # uthai.joints_move( [0, -0.2,    0.1, 0.1, -0.2, 0.05, 0, 0.2, -0.25, 0.3, -0.1, 0.05], 0.2)

        rospy.sleep(5)
        # uthai.joints_move( [0, -0.11, 0.1, 0.1, -0.2, 0.05, 0, 0.25, -0.25, 0.3, -0.1, 0.05], 0.2)

        # uthai.joints_move( [0, -0.2, -0.1, 0.1, -0.2, 0.3, 0, 0.3, -0.25, 0.3, -0.1, 0.3], 0.8)
        # uthai.joints_move( [0, -0.25, -0.1, 0.2, -0.1, 0.0, 0, 0.11, 0.5, 1.0, -0.5, 0.0], 0.2)

        # uthai.joints_move( [0, -0.11, -0.25, 0.5, -0.35, -0.3, 0, 0.11, -0.1, 0.2, -0.15, -0.3], 0.8)
        # uthai.joints_move( [0, -0.11, 0.0, 0.2, -0.15, -0.15, 0, 0.2, -0.5, 1.0, -0.5, -0.25], 0.4)
        # uthai.joints_move( [0, -0.11, 0.1, 0.1, -0.1, 0.05, 0, 0.25, -0.2, 0.3, -0.05, 0.05], 0.3)
        # rospy.sleep(2)

        # uthai.joints_move( [0, -0.11, -0.25, 0.3, -0.1, -0.1, 0, 0.15, -0.25, 0.3, -0.1, -0.1], 0.1)




        # uthai.joints_move( [0, -0.11, -0.1, 0.2, -0.1, -0.2, 0, 0.11, -0.1, 0.2, -0.1, -0.2], 0.5)
        # uthai.joints_move( [0, -0.11, -0.4, 0.7, -0.4, 0.2, 0, 0.11, -0.1, 0.2, -0.12, 0.2], 1.5)




        # uthai.joints_move( [0, -0.09, 0, 0, 0, 0.12, 0, 0.09, 0, 0, 0, 0.12], 1.0)






        # left2
        # uthai.joints_move( [-0.07163536881347156, -0.12509459927128616, -0.04276738436625168, -0.002138369218312584, 0.08125803029587819, 0.10798764552478549, -0.019245322964813256, -0.019245322964813256, 0.05132086123950201, 0.00534592304578146, -0.01603776913734438, 0.09836498404237887],1.0)
        # uthai.joints_move( [0, 0.06, 0, 0, 0, -0.06, 0, 0.06, 0, 0, 0, -0.06], 0.5)
        # move to right
        # uthai.joints_move( [-0.07377373803178415, 0.07270455342262785, 0.07805047646840932, -0.00534592304578146, -0.05559759967612718, -0.052390045848658306, -0.019245322964813256, 0.19459159886644514, 0.1593085067642875, 0.00534592304578146, -0.1315097069262239, -0.07591210725009673], 1.0)
    
        
        # rospy.sleep(1)
        # uthai.joints_move([0.0]*12, 1.0)
        # rospy.sleep(1)
        # rospy.sleep(1)
        # uthai.joints_move([0.0]*12, 1.0)
        # rospy.sleep(1)
        # rospy.sleep(2)
    # rospy.sleep(2.0)
    # with open('MoveCoML.csv', 'r') as csvfile:
    #     spamreader = csvfile.read().split('\n')
    #     for row in spamreader:
    #         q_set = map(float, row.split(','))
    #         uthai.joint_move(q_set, 0.1)
    # uthai.joints_move([0.0]*12, 5.0)

    rospy.spin()


if __name__ == '__main__':
    main()
