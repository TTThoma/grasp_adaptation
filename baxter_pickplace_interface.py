import struct
import copy

import rospy
import geometry_msgs.msg
import std_msgs.msg
import baxter_core_msgs.srv
import baxter_interface

from utils import *


class PickAndPlace(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=False):
        self.starting_joint_angles = {'left_w0': 0.854067711823217, 'left_w1': 1.353323798747246, 'left_w2': 0.20707233915603135, 'left_e0': -1.2464470809159323, 'left_e1': 1.521728639255441, 'left_s0': 0.7458398104843669, 'left_s1': -0.6782830967604189}
#                                      {'left_w0': 0.7699334738586732,
#                                       'left_w1': 1.0561707149249662,
#                                       'left_w2': -0.5799010702334275,
#                                       'left_e0': -1.1224124454028832,
#                                       'left_e1': 1.8723043286991299,
#                                       'left_s0': 0.5794635695064052,
#                                       'left_s1': -0.8376263577834537}
        self.middle_joint_angles = {'left_w0': 0.7633833981383319, 'left_w1': 1.1988001389610117, 'left_w2': -0.2046613444788766, 'left_e0': -1.401196365622845, 'left_e1': 1.921777589314542, 'left_s0': 0.6934488680254683, 'left_s1': -0.8544163902726412}
#                                    {'left_w0': 0.7511013559549635,
#                                     'left_w1': 1.030009435085784,
#                                     'left_w2': -0.8309864457708613,
#                                     'left_e0': -1.092685751801997,
#                                     'left_e1': 1.825029051590004,
#                                     'left_s0': -0.22994278634062315,
#                                     'left_s1': -0.8339010534647127}
        self.ready_joint_angles = {'left_w0': 0.6598654670140606, 'left_w1': 1.0889851521710656, 'left_w2': -0.6112163594989781, 'left_e0': -1.455345256022783, 'left_e1': 2.091634206576868, 'left_s0': 0.38961735358609306, 'left_s1': -0.9983541993960502}
#                                   {'left_w0': 0.7681817732537244,
#                                    'left_w1': 1.1958299581009344, 
#                                    'left_w2': -0.9180607141559838, 
#                                    'left_e0': -0.999258925850091, 
#                                    'left_e1': 1.4962351371242208, 
#                                    'left_s0': -0.32994278634062315, 
#                                    'left_s1': -0.6943242529357878}
        
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, baxter_core_msgs.srv.SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
#         print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
#         print("Enabling robot... ")
        self._rs.enable()
        
        self._gripper.set_holding_force(0.1)
        self._gripper.set_moving_force(0.1)
        self._limb.set_joint_position_speed(0.9)

    def move_to_start(self, start_angles=None, open_gripper=True):
#         print("Moving the {0} arm to start pose...".format(self._limb_name))
        if start_angles is None:
            start_angles = self.starting_joint_angles
        self._guarded_move_to_joint_position(start_angles)
        if open_gripper:
            self.gripper_open()
        rospy.sleep(0.01)
#         print("Running. Ctrl-c to quit")
    
    def move_to_middle(self, middle_angles=None, open_gripper=True):
#         print("Moving the {0} arm to middle pose...".format(self._limb_name))
        if middle_angles is None:
            middle_angles = self.middle_joint_angles
        self._guarded_move_to_joint_position(middle_angles)
        if open_gripper:
            self.gripper_open()
        rospy.sleep(0.01)
#         print("Running. Ctrl-c to quit")
        
    def move_to_ready(self, ready_angles=None, open_gripper=True):
#         print("Moving the {0} arm to ready pose...".format(self._limb_name))
        if ready_angles is None:
            ready_angles = self.ready_joint_angles
        self._guarded_move_to_joint_position(ready_angles)
        if open_gripper:
            self.gripper_open()
        rospy.sleep(0.01)
#         print("Running. Ctrl-c to quit")
    
    def ik_request(self, pose):
        hdr = std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = baxter_core_msgs.srv.SolvePositionIKRequest()
        ikreq.pose_stamp.append(geometry_msgs.msg.PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()

    def gripper_close(self):
        self._gripper.close()

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        
        approach_drection = - rotate_quaternion_vector(approach.orientation, [0., 0., 1.])
        approach.position.x += self._hover_distance * approach_drection[0]
        approach.position.y += self._hover_distance * approach_drection[1]
        approach.position.z += self._hover_distance * approach_drection[2]
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = geometry_msgs.msg.Pose()
        retract_drection = - rotate_quaternion_vector(current_pose['orientation'], [0., 0., 1.])
        ik_pose.position.x = current_pose['position'].x + self._hover_distance * retract_drection[0]
        ik_pose.position.y = current_pose['position'].y + self._hover_distance * retract_drection[1]
        ik_pose.position.z = current_pose['position'].z + self._hover_distance * retract_drection[2]
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
        return joint_angles

    def move(self, pose):
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
        
    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        servo_result = self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        if servo_result:
            self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()