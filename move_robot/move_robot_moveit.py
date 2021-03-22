import rospy
from moveit_msgs.msg import ExecuteTrajectoryGoal
import numpy as np
import math
import actionlib
from scipy.spatial.transform.rotation import Rotation
from bin_picking.msg import MoveRobotAction, MoveRobotGoal


class MoveRobotMoveIt:
    def __init__(self, create_node=False):
        if create_node:
            rospy.init_node("moveit_test", anonymous=True)
        self.client = actionlib.SimpleActionClient("bin_picking_moveit_interface", MoveRobotAction)
        self.client.wait_for_server()
        #self.home_pose_l = [35, -300, 300, 0, 0, -0.8] # old
        self.home_pose_l = [-0.035, 0.300, 0.300, 0, 0, -0.8]
        #self.home_pose_gripper = [-60, -60, -110, -100, 90, -60] # old
        #self.home_pose_gripper = [0.060, 0.060, -0.110, -100, 90, -60]
        #self.home_pose_suction = [-60, -60, -110, -190, -70, 100] # old
        #self.home_pose_suction = [0.060, 0.060, -0.110, -190, -70, 100]
        self.home_pose_gripper = [-60, -60, -110, -100, 90, -60]
        self.home_pose_suction = [-60, -60, -110, 170, -70, 100]
        self.move_out_of_view_pose = [-150, -60, -110, 170, -70, 100]
        self.default_orientation = [0, 0, 0]
        self.gripper_tcp = [0, 0, 0.201, 0, 0, 0]
        self.suction_tcp = [-0.193, 0, 0.08, 0, -np.pi/2, 0]

        self.camera_pose_gripper = [-60, -60, -110, -100, -90, -75]
        self.camera_pose_suction = [-5, -40, -100, -140, 0, -170]

        self.pcb_singularity_avoidance = [-70, -70, -107, -180, -147, 90]

        self.cover_closed = 20
        self.box_closed = 3

        #Part drop locations:
        #self.white_cover_drop = [350, -400, 100, 2.89, 1.21, 0] # old
        #self.white_cover_drop = [-0.350, 0.400, 0.300, -0.61, 1.48, 0.62]
        # New frames are R_z(pi) * current_transform * R_z(pi/2) * R_y(-pi/2)
        self.white_cover_drop = [-0.350, 0.400, 0.300, 0.61, 1.48, -0.61] # this one is calibrated for new frames
        self.black_cover_drop = [200, -250, 100, 2.89, 1.21, 0]
        self.blue_cover_drop = [-50, -250, 100, 2.89, 1.21, 0]
        self.bottom_cover_drop = [-150, -350, 100, 2.89, 1.21, 0]
        self.pcb_drop = [-250, -450, 100, 2.89, 1.21, 0]

        rospy.loginfo("Move Robot interface ready")

    def movej(self, pose, acceleration=1.0, velocity=0.1, degrees=True):
        pose_local = pose.copy()
        if degrees:
            for i in range(6):
                pose_local[i] = math.radians(pose_local[i])
        goal = MoveRobotGoal()
        goal.action = "joint"
        goal.joint_goal.positions = pose_local
        goal.joint_goal.velocities = np.repeat(velocity, 6)
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def movel(self, pose, acceleration=1.0, velocity=0.2):
        pose_local = pose.copy()
        quaternion = Rotation.from_rotvec(pose_local[3:]).as_quat()
        goal = MoveRobotGoal()
        goal.action = "cartesian"
        goal.cartesian_goal.position.x = pose_local[0]
        goal.cartesian_goal.position.y = pose_local[1]
        goal.cartesian_goal.position.z = pose_local[2]
        goal.cartesian_goal.orientation.x = quaternion[0]
        goal.cartesian_goal.orientation.y = quaternion[1]
        goal.cartesian_goal.orientation.z = quaternion[2]
        goal.cartesian_goal.orientation.w = quaternion[3]
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def movel2(self, location, orientation, acceleration=1.0, velocity=0.2):
        self.movel(np.concatenate(location, orientation))

    def set_tcp(self, pose):
        # TODO: Implement this
        pass

    def move_to_home_suction(self, speed=1.0):
        self.movej(self.home_pose_suction, acceleration=1.0, velocity=speed)

    def move_to_home_gripper(self, speed=1.0):
        self.movej(self.home_pose_gripper, acceleration=1.0, velocity=speed)

    def move_to_home_l(self, speed=1.0):
        self.movel(self.home_pose_l, acceleration=1.0, velocity=speed)

    def move_out_of_view(self, speed=2.0):
        self.movej(self.move_out_of_view_pose, acceleration=1.0, velocity=speed)

    def open_gripper(self):
        goal = MoveRobotGoal()
        goal.action = "open_gripper"
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def close_gripper(self, width=0, speed=5, lock=False, gripping_box=False):
        goal = MoveRobotGoal()
        goal.action = "close_gripper"
        goal.gripper_width = width
        goal.gripper_speed = speed
        goal.gripper_lock = lock
        goal.gripper_grip_box = gripping_box
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def grasp_cover(self):
        self.close_gripper(self.cover_closed)

    def grasp_box(self):
        self.close_gripper(self.box_closed, gripping_box=True)

    def enable_suction(self):
        goal = MoveRobotGoal()
        goal.action = "suction_on"
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def disable_suction(self):
        goal = MoveRobotGoal()
        goal.action = "suction_off"
        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == "__main__":
    mr = MoveRobotMoveIt(create_node=True)
    #mr.close_gripper(10)
    #mr.open_gripper()
    #mr.move_to_home_gripper()
    #mr.move_to_home_suction()
    #mr.move_out_of_view()
    mr.movel(mr.white_cover_drop)