import rospy
import rospy
import intera_interface
from intera_interface import (
    SimpleClickSmartGripper,
    get_current_gripper_interface,
)

class GrabWolf(object):
   	def __init__(self, limb="right"):
       		self._limb_name = limb # string
        	self._limb = intera_interface.Limb(limb)
        	self._gripper = get_current_gripper_interface()
		self._head_display = intera_interface.HeadDisplay()
        	# verify robot is enabled
        	print("Getting robot state... ")
        	self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        	self._init_state = self._rs.state().enabled
        	print("Enabling robot... ")
       		self._rs.enable()
    
	def move_to_start(self, start_angles=None):
        	print("Moving the {0} arm to start pose...".format(self._limb_name))
        	if not start_angles:
            		start_angles = dict(zip(self._joint_names, [0]*7))
        	self._guarded_move_to_joint_position(start_angles)
        	self.gripper_open()

	def _guarded_move_to_joint_position(self, joint_angles, timeout=5.0):
        	if rospy.is_shutdown():
           		return
        	if joint_angles:
            		self._limb.move_to_joint_positions(joint_angles,timeout=timeout)
        	else:
            		rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    
	def gripper_open(self):
	        self._gripper.set_ee_signal_value('grip', True)
	        rospy.sleep(1.0)
    
	def gripper_close(self):
	        self._gripper.set_ee_signal_value('grip', False)
	        rospy.sleep(1.0)

	def move_to_wolf_position(self, wolf_position = None):
		print("Grabbing Wolf")
		if not wolf_position:
			wolf_position = {'right_j6': -1.442673828125, 'right_j5': -1.3330966796875, 'right_j4': 1.89010546875, 'right_j3': -1.324009765625, 'right_j2': -0.713978515625, 'right_j1': 1.0837001953125, 'right_j0': -0.952453125}
		self._limb.move_to_joint_positions(wolf_position, timeout = 7.0)
	
	def grab_wolf(self, grab_position = None):
		if not grab_position:
			grab_position = {'right_j6': -1.340985, 'right_j5': -1.073005, 'right_j4': 1.850688, 'right_j3': -1.035657, 'right_j2': -0.592413, 'right_j1': 0.955134, 'right_j0': -0.990265}
		self._limb.move_to_joint_positions(grab_position, timeout = 2.0)

	def move_to_handoff(self, handoff_position = None):
		if not handoff_position:
			handoff_position = {'right_j6': -2.395357, 'right_j5': 0.464996, 'right_j4': -0.0131210, 'right_j3': -1.310717, 'right_j2': -0.736825, 'right_j1': 0.711936, 'right_j0': -0.475967}
		self._limb.move_to_joint_positions(handoff_position, timeout = 5.0)

def main():
	rospy.init_node("grab_wolf")
	limb = "right"
	starting_joint_angles = {'right_j0': -0.041662954890248294,
                             'right_j1': -1.0258291091425074,
                             'right_j2': 0.0293680414401436,
                             'right_j3': 2.17518162913313,
                             'right_j4':  -0.06703022873354225,
                             'right_j5': 0.3968371433926965,
                             'right_j6': 1.7659649178699421}
	wg = GrabWolf(limb)
	wg._head_display.display_image('welcome.png')
	wg.move_to_start(starting_joint_angles)
	for move in range(1):
		wg.move_to_wolf_position()
		wg.grab_wolf()
		wg.gripper_close()
		wg.move_to_handoff()
		wg.gripper_open()
		wg.move_to_start(starting_joint_angles)

if __name__ == '__main__':
    main()

		
