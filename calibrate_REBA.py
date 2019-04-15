from thesis_functions import (move_vertical, move_to_start_position)
import rospy

rospy.init_node('test_bobby')
move_to_start_position()
for move in range(3):
	move_vertical('down')
	rospy.sleep(2.0)
	move_vertical('up')
	rospy.sleep(2.0)
