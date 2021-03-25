import rospy
import actionlib
from bin_picking.msg import PickObjectAction, PickObjectGoal
import cv2
from cv_bridge import CvBridge
import numpy as np
import cv_bridge

rospy.init_node("picking_node_tester")

client = actionlib.SimpleActionClient("pick_object", PickObjectAction)
print("waiting for server ready")
client.wait_for_server()
goal = PickObjectGoal()
test_img = cv2.imread("testing_resources/background_test.png")
bridge = CvBridge()

goal.goal_msg = "test"
goal.mask = bridge.cv2_to_imgmsg(test_img)
goal.reference_img = bridge.cv2_to_imgmsg(test_img)
goal.depth_img = bridge.cv2_to_imgmsg(test_img)

print("sending request")
client.send_goal(goal)
client.wait_for_result()
result = client.get_result()

print(result)