import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb
import intera_interface




def move_vertical(direction):
	#rospy.init_node("move_up")
	# create kdl frame from relative pose
	limb = Limb()
	traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
        wpt_opts = MotionWaypointOptions(max_linear_speed=0.6,
                                         max_linear_accel=0.6,
                                         max_rotational_speed=1.57,
                                         max_rotational_accel=1.57,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
	endpoint_state = limb.tip_state('right_hand')
	pose = endpoint_state.pose
	rot = PyKDL.Rotation.RPY(0,0,0)
	if direction is 'up':
		trans = PyKDL.Vector(0,-1,0)
	if direction is 'down':
		trans = PyKDL.Vector(0,1,0)
	f2 = PyKDL.Frame(rot, trans)
	# and convert the result back to a pose message
	pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
	poseStamped = PoseStamped()
	poseStamped.pose = pose
	joint_angles = limb.joint_ordered_angles()
	waypoint.set_cartesian_pose(poseStamped, 'right_hand', joint_angles)
	traj.append_waypoint(waypoint.to_msg())
	result = traj.send_trajectory(timeout=None)

def move_to_start_position():
        limb = Limb()
        traj = MotionTrajectory(limb = limb)
        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.5,
                                         max_joint_accel=0.5)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
        joint_angles = limb.joint_ordered_angles()
        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())
        waypoint.set_joint_angles(joint_angles = [0.36014453125,-1.551955078125, 2.7190439453125,-1.438056640625,-0.48725, 0.11405078125,-2.4699443359375])
        traj.append_waypoint(waypoint.to_msg())
        result = traj.send_trajectory(timeout=None)

#def move_to_anthro_M
#def move_to_anthro_F


