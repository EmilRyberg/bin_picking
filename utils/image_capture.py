import cv2
import rospy
from vision_lib.ros_camera_interface import ROSCamera
from bin_picking_lib.move_robot.move_robot_moveit import MoveRobotMoveIt

rospy.init_node("capture_node")

move_robot = MoveRobotMoveIt()
camera = ROSCamera()
move_robot.move_out_of_view()

a = "4_"
i=0
while True:
    img = camera.get_image()
    cv2.imwrite("images/"+a+str(i)+".png", img)
    i+=1
    c=1