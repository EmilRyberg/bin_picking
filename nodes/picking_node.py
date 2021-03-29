import rospy
import actionlib
from vision.surface_normal import SurfaceNormals
import cv2
import numpy as np
from bin_picking.msg import PickObjectAction, PickObjectFeedback, PickObjectResult
from cv_bridge import CvBridge
from move_robot.move_robot_moveit import MoveRobotMoveIt
from ros_camera_interface import ROSCamera
from testing_resources.find_objects import FindObjects, ObjectInfo

class PickingNode:
    def __init__(self, testing=False):
        rospy.init_node("picking_node")


        self.action_server = actionlib.SimpleActionServer("pick_object", PickObjectAction, self.callback, auto_start=False)

        self.surface_normals = SurfaceNormals()
        self.bridge = CvBridge()
        self.move_robot = MoveRobotMoveIt()

        if testing:
            self.camera = ROSCamera()
            self.background_img = cv2.imread("testing_resources/background_test.png")
            self.object_finder = FindObjects(self.background_img, [0, 0, 400, 400])

        self.action_server.start()
        print("sever started")

    def callback(self, goal):
        print("entered callback")
        feedback = PickObjectFeedback()
        feedback.status = "executing"
        self.action_server.publish_feedback(feedback)
        command = goal.command
        if command == "move_out_of_view":
            self.move_robot.move_out_of_view()
        elif command == "test":
            self.move_robot.move_out_of_view()
            img = self.camera.get_image()
            depth_img = self.camera.get_depth()
            object_to_pick = self.object_finder.find_objects(img, debug=False)[0]
            mask = object_to_pick.mask_full
            #cv2.imwrite("testing_resources/img.png", img)
            #cv2.imwrite("testing_resources/mask.bmp", mask)
            #np.save("testing_resources/depth_img.npy", depth_img)
            #cv2.imshow("a", object_to_pick.object_img_cutout_cropped)
            #cv2.waitKey()
            center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(mask, depth_img, self.background_img, 0)
            self.move_robot.move_to_home_gripper(speed=3)
            self.move_robot.movel([0, -300, 300, 0, np.pi, 0], velocity=0.8, use_mm=True)
            approach_center = center + 200 * normal_vector
            pose_approach = np.concatenate((approach_center, rotvec))
            self.move_robot.movel(pose_approach, use_mm=True)
            pose_pick = np.concatenate((center - 14 * normal_vector, rotvec))
            self.move_robot.close_gripper(50)
            self.move_robot.movel(pose_pick, velocity=0.1, use_mm=True)
            gripper_close_distance = 40
            self.move_robot.close_gripper(gripper_close_distance, speed=0.5, lock=True)
            self.move_robot.movel2([center[0], center[1], 100], rotvec, use_mm=True)
            print("test done")
        elif command == "pick_object":
            mask = self.bridge.imgmsg_to_cv2(goal.mask, desired_encoding="passthrough")
            reference_img = self.bridge.imgmsg_to_cv2(goal.reference_img, desired_encoding="passthrough")
            depth_img = self.bridge.imgmsg_to_cv2(goal.depth_img, desired_encoding="passthrough")
            center, rotvec, normal_vector, relative_angle_to_z, short_vector = self.surface_normals.get_gripper_orientation(mask, depth_img, reference_img, 0)
            self.move_robot.move_to_home_gripper(speed=3)
            self.move_robot.movel([0, -300, 300, 0, np.pi, 0], velocity=0.8, use_mm=True)
            approach_center = center + 200 * normal_vector
            pose_approach = np.concatenate((approach_center, rotvec))
            self.move_robot.movel(pose_approach, use_mm=True)
            pose_pick = np.concatenate((center - 14 * normal_vector, rotvec))
            self.move_robot.close_gripper(50)
            self.move_robot.movel(pose_pick, velocity=0.1, use_mm=True)
            gripper_close_distance = 20
            self.move_robot.close_gripper(gripper_close_distance, speed=0.5, lock=True)
            self.move_robot.movel2([center[0], center[1], 100], rotvec, use_mm=True)
            self.move_robot.movel2([center[0], center[1], 300], [0, np.pi, 0], use_mm=True)
        elif command == "place_object":
            self.move_robot.movel2(goal.position, [0, np.pi, 0], use_mm=True)
            self.move_robot.open_gripper()
        else:
            rospy.logerr("received invalid command" + command)
            raise Exception("received invalid command" + command)

        result = PickObjectResult()
        result.success = True
        self.action_server.set_succeeded(result=result)
        return


if __name__ == "__main__":
    server = PickingNode(testing=True)