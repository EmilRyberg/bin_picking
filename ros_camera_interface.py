import numpy as np
from PIL import Image as pimg
from camera_interface import CameraInterface
import actionlib
from bin_picking.msg import MoveRobotAction, MoveRobotGoal
from cv_bridge import CvBridge


class ROSCamera(CameraInterface):
    def __init__(self):
        self.client = actionlib.SimpleActionClient("bin_picking_moveit_interface", MoveRobotAction)
        self.client.wait_for_server()
        self.bridge = CvBridge()

    def get_image(self):
        goal = MoveRobotGoal()
        goal.action = "get_image"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        np_img = self.bridge.compressed_imgmsg_to_cv2(result.rgb_compressed, desired_encoding="bgr8")
        print("np_img", np_img.shape)
        print("np_img", np_img)
        return np_img

    def get_depth(self):
        goal = MoveRobotGoal()
        goal.action = "get_depth"
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        np_img = self.bridge.compressed_imgmsg_to_cv2(result.depth_compressed, desired_encoding="passthrough")
        print("np_img", np_img.shape)
        print("np_img", np_img)
        return np_img


if __name__ == "__main__":
    camera = ROSCamera()
    img = camera.get_image()
    depth = camera.get_depth()