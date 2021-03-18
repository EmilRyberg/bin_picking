import rospy
import actionlib
from std_msgs.msg import String
from controller.enums import PartEnum, PartCategoryEnum
from vision.surface_normal import SurfaceNormals
from PIL import Image as pimg
import cv2
import numpy as np
import random
from bin_picking.msg import PickObjectAction, PickObjectActionGoal

class PickingNode:
    def __init__(self):
        rospy.init_node("picking_node", anonymous=True)
        self.action_server = actionlib.SimpleActionServer("pick_object_server", PickObjectAction, self.pick_part)

        self.surface_normals = SurfaceNormals()

    def pick_part(self, mask, reference_img, depth_img):

        print("picking part")
        """
        center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(
            mask, depth_img, reference_img, 0)
        self.move_robot.set_tcp(self.move_robot.gripper_tcp)
        self.move_robot.move_to_home_gripper(speed=3)
        self.move_robot.movel([0, -300, 300, 0, np.pi, 0], vel=0.8)
        approach_center = center + 200 * normal_vector
        pose_approach = np.concatenate((approach_center, rotvec))
        self.move_robot.movel(pose_approach)
        pose_pick = np.concatenate((center - 14 * normal_vector, rotvec))
        self.move_robot.close_gripper(40)
        self.move_robot.movel(pose_pick, vel=0.1)
        gripper_close_distance = 0
        self.move_robot.close_gripper(gripper_close_distance, speed=0.5, lock=True)
        self.move_robot.movel2([center[0], center[1], 100], rotvec)
        if not self.has_object_between_fingers(15 / 1000.0):  #dropped part, 18 is part width
            self.unsuccessful_grip_counter += 1
            self.move_robot.open_gripper()
            self.move_robot.movel([center[0], center[1], 300, 0, 3.14, 0], vel=0.8)
            self.move_robot.move_out_of_view()
        else: #good grip
            self.move_robot.movel([center[0], center[1], 200, 0, np.pi, 0], vel=0.8)
            # self.move_robot.movel([200, -200, 50, 0, np.pi, 0])
            self.move_to_drop(mask["part"])
            self.move_robot.open_gripper()
            self.move_robot.move_to_home_gripper(speed=3)
        """


if __name__ == "__main__":
    server = PickingNode()