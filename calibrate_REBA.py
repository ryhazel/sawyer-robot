from thesis_functions import (move_vertical, move_to_start_position, gripper_open, gripper_close)
import rospy
import rosbag
import numpy as np
import scipy.io
from intera_interface import Limb
import threading
import time

rospy.init_node('test_bobby')

move_to_start_position()

limb = Limb()

pose_data = []

#start thread to sample cartesian position (x,y,z) of the robot end effector
def collect_data():
	t = threading.Timer(0.5, collect_data)
	t.start()
	pose = limb.endpoint_pose()
	pose_data.append(([pose['position'].x, pose['position'].y, pose['position'].z]))	
	#cancel thread when count reaches 3	
	if count == 3:
		t.cancel()


time1 = rospy.get_time()

count = 0

#start collect_data() and begin moving the robot
collect_data()
for move in range(3):
	move_vertical('down')
	rospy.sleep(2.0)
	move_vertical('up')
	rospy.sleep(2.0)
	count += 1
	print(count)

time2 = rospy.get_time()

pose_data_array = np.array(pose_data)
np.savetxt("test.csv", pose_data_array, delimiter = ",")
print(time2-time1)
print(pose_data_array)
