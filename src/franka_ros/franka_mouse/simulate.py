# Input from /spacenav/joy
#   message type sensor_msgs/Joy
#       Contains header, float32[] axes, int32[] buttons
# Output to /cartesian_pose
#   message type gemoetry_msgs/PoseStamped
#       Contains header, geometry_msgs/Pose pose made up of
#           geometry_msgs/Point position
#               float64 x
#               float64 y
#               float64 z
#           geometry_msgs/Quaterinion orientation
#
# Utilises https://github.com/justagist/franka_panda_description and https://github.com/justagist/franka_ros_interface

import rospy
import sys
import actionlib
import franka_gripper.msg
from pynput import keyboard
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation

scale_factor = 0.05
input_msg = Joy()
key = 0
curr_pos = PoseStamped()
is_pose = 0

def move_gripper(gripperWidth):
	#Initialise action client to move gripper
	moveClient = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

	#Wait until server has started
	moveClient.wait_for_server()
	#Create a goal with passed-in width to the gripper
	goal = franka_gripper.msg.MoveGoal(width=gripperWidth, speed=1.0)

	moveClient.send_goal(goal)

	moveClient.wait_for_result()

	return moveClient.get_result()

def grasp_gripper(gripperWidth):
	#Initialise action client to move gripper
	moveClient = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

	#Wait until server has started
	moveClient.wait_for_server()
	#Create a goal with passed-in width to the gripper
	graspEpsilon = franka_gripper.msg.GraspEpsilon(inner=0.1,outer=0.1)
	goal = franka_gripper.msg.GraspGoal(width=gripperWidth, epsilon=graspEpsilon, speed=1.0,force=20)

	moveClient.send_goal(goal)

	#moveClient.wait_for_result()
	rate = rospy.Rate(5)
	rate.sleep()

	return moveClient.get_result()

def stop():
	global curr_pos
	ctrl_msg = PoseStamped()
	ctrl_msg.pose.position.x = curr_pos.pose.position.x
	ctrl_msg.pose.position.y = curr_pos.pose.position.y
	ctrl_msg.pose.position.z = curr_pos.pose.position.z
	ctrl_msg.pose.orientation.x = curr_pos.pose.orientation.x
	ctrl_msg.pose.orientation.y = curr_pos.pose.orientation.y
	ctrl_msg.pose.orientation.z = curr_pos.pose.orientation.z
	ctrl_msg.pose.orientation.w = curr_pos.pose.orientation.w
	return ctrl_msg

def control_callback(data):
	global input_msg
	global key
	input_msg = data
	key = 1


def pose_callback(data):
	global curr_pos
	curr_pos = data

def move_from_file():
	# Reads an instruction file located at data/Run.txt and moves the robot
	moveFile = open("data/Run.txt", "r")
	instructions = moveFile.readlines()
	converting = True
	lineNumber = 0
	desiredPose = PoseStamped()
	rate = rospy.Rate(5)
	print("Starting Read\n")

	while(converting):
		if (lineNumber == len(instructions)):
			converting = False
			print("Run Completed\n")
		else:
			line = instructions[lineNumber]
			if(line == "Run:"):
				# Move to the first line of the run
				lineNumber = lineNumber+1
			elif(line[0] == "p"):
				# Check desired pose position - there shouldn't be any individual orientation values for string
				desiredPose.pose.position.x = float(instructions[lineNumber+1].translate({ord(i): None for i in 'x :'}))
				desiredPose.pose.position.y = float(instructions[lineNumber+2].translate({ord(i): None for i in 'y :'}))
				desiredPose.pose.position.z = float(instructions[lineNumber+3].translate({ord(i): None for i in 'z :'}))

				# Check desired pose orientation
				desiredPose.pose.orientation.x = float(instructions[lineNumber+5].translate({ord(i): None for i in 'x :'}))
				desiredPose.pose.orientation.y = float(instructions[lineNumber+6].translate({ord(i): None for i in 'y :'}))
				desiredPose.pose.orientation.z = float(instructions[lineNumber+7].translate({ord(i): None for i in 'z :'}))
				desiredPose.pose.orientation.w = float(instructions[lineNumber+8].translate({ord(i): None for i in 'w :'}))

				print("Moving To Pose\n")

				# Publish the pose then wait to ensure the system isn't queueing too badly
				posePub.publish(desiredPose)
				rate.sleep()

				# Move to the next line
				lineNumber = lineNumber+9
			elif(line[0] == "G"):
				print("Closing gripper")
				grasp_gripper(0.01)
				lineNumber = lineNumber+1
			elif(line[0] == "R"):
				print("Opening gripper")
				move_gripper(0.08)
				lineNumber = lineNumber+1
			else:
				lineNumber = lineNumber+1
				print("Huh, something weird in the file at line "+str(lineNumber))

def joystick_control():
	global key
	running = True
	#Initialise Publisher and Subscriber
	poseSub = rospy.Subscriber('/cartesian_pose', PoseStamped, pose_callback)
	inputSub = rospy.Subscriber('/spacenav/joy', Joy, control_callback)

	#Open the record file
	logFile = open("data/pickAndPlaceRuns.txt", "a+")
	logFile.write("Run\n")
	logFile.close()
	
	firstRun = True
	
	
	

	print("Launching Joystick Interface, press z to end\n")

	while (running):
		try:
			if key == 1:
				ctrl_msg = PoseStamped()
				rate = rospy.Rate(5)
				global scale_factor
				global curr_pos
				rotation_factor = 10

				if not rospy.is_shutdown():
					#orgAngleEuler = orgAngle.as_euler('xyz',degrees=True)
					#rot = Rotation.from_euler('xyz', [(input_msg.angular.x*5)+orgAngleEuler[0],(input_msg.angular.y*5)+orgAngleEuler[1],(input_msg.angular.x*5)+orgAngleEuler[2]], degrees=True)
					if firstRun:
						rate.sleep()
						#print(curr_pos)
						position_x = curr_pos.pose.position.x
						position_y = curr_pos.pose.position.y
						position_z = curr_pos.pose.position.z
						
						orientation_x = curr_pos.pose.orientation.x
						orientation_y = curr_pos.pose.orientation.y
						orientation_z = curr_pos.pose.orientation.z
						orientation_w = curr_pos.pose.orientation.w
						firstRun = False

					if (input_msg.axes[0] != 0.0):
						ctrl_msg.pose.position.x = curr_pos.pose.position.x + (input_msg.axes[0]*scale_factor)
						position_x = ctrl_msg.pose.position.x
					else:
						ctrl_msg.pose.position.x = position_x
					if (input_msg.axes[1] != 0.0):
						ctrl_msg.pose.position.y = curr_pos.pose.position.y + (input_msg.axes[1]*scale_factor)
						position_y = ctrl_msg.pose.position.y
					else:
						ctrl_msg.pose.position.y = position_y
					if (input_msg.axes[2] != 0.0):
						ctrl_msg.pose.position.z = curr_pos.pose.position.z + (input_msg.axes[2]*scale_factor)
						position_z = ctrl_msg.pose.position.z
					else:
						ctrl_msg.pose.position.z = position_z
					#print(curr_pos)
					#if ((input_msg.axes[3] < 0.2) or (input_msg.axes[4] < 0.2)  or (input_msg.axes[5] < 0.2)):
					if False:
						orgAngle = Rotation.from_quat([curr_pos.pose.orientation.x,curr_pos.pose.orientation.y,curr_pos.pose.orientation.z,curr_pos.pose.orientation.w])
						rot = Rotation.from_euler('xyz', [(input_msg.axes[3]*rotation_factor),(-input_msg.axes[4]*rotation_factor),(-input_msg.axes[5]*rotation_factor)], degrees=True)
						combined = orgAngle*rot
						quaternion = combined.as_quat()
						#print(quaternion)
						ctrl_msg.pose.orientation.x = quaternion[0]
						ctrl_msg.pose.orientation.y = quaternion[1]
						ctrl_msg.pose.orientation.z = quaternion[2]
						ctrl_msg.pose.orientation.w = quaternion[3]
						orientation_x = quaternion[0]
						orientation_y = quaternion[1]
						orientation_z = quaternion[2]
						orientation_w = quaternion[3]
					else:
						ctrl_msg.pose.orientation.x = orientation_x
						ctrl_msg.pose.orientation.y = orientation_y
						ctrl_msg.pose.orientation.z = orientation_z
						ctrl_msg.pose.orientation.w = orientation_w
					#print (ctrl_msg)
					posePub.publish(ctrl_msg)
					logFile = open("data/pickAndPlaceRuns.txt", "a")
					logFile.write(str(ctrl_msg.pose)+"\n")

					if (input_msg.buttons[0] == 1):
						result = grasp_gripper(0)
						logFile.write("Grasp\n")
					elif (input_msg.buttons[1] == 1):
						result = move_gripper(0.08)
						logFile.write("Release\n")

					logFile.close()
					rate.sleep()
				key = 0
		except KeyboardInterrupt:
			sys.exit()
		except Exception as e:
			print(e)

def main():
	running = True
	rospy.init_node('Simulator_Interface', anonymous=True)
	while(running):
		check = input("Press r to run from file, or n to record new run\n")
		if (check == 'r'):
			print()
			move_from_file()
		elif (check == 'n'):
			joystick_control()
		else:
			print("Make sure it's r or n, lower case, nothing else, please and thank you\n")


if __name__ == '__main__':
	try:
		# Start up the publisher and pose subscriber and run the main function
		posePub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=1)

		main()
	except rospy.ROSInterruptException:
		# rospy.ROSInterruptException
		pass
	except KeyboardInterrupt:
		sys.exit()
	except Exception as e:
		print(e)
