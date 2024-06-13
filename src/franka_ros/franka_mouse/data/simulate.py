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
import imitation
from imitation.data import serialize
from imitation.data.types import Trajectory
import numpy as np
import datasets
from pathlib import Path
import numpy as np
import gymnasium as gym
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3 import PPO
#from stable_baselines3.ppo import MultiInputPolicies

from imitation.algorithms import bc
from imitation.data import rollout
from imitation.data import serialize
from imitation.data.wrappers import RolloutInfoWrapper
from imitation.policies.serialize import load_policy
from imitation.util.util import make_vec_env
import franka_env
from imitation.data.types import Trajectory
from gymnasium.wrappers import FlattenObservation
from imitation.data.wrappers import RolloutInfoWrapper
from stable_baselines3.common.vec_env.base_vec_env import VecEnv, VecEnvStepReturn, VecEnvWrapper
from stable_baselines3.common.vec_env import DummyVecEnv
from imitation.util.util import make_vec_env
from stable_baselines3.common.policies import MultiInputActorCriticPolicy
from imitation.algorithms.adversarial.gail import GAIL
from imitation.rewards.reward_nets import BasicRewardNet
from imitation.util.networks import RunningNorm
from imitation.data.types import TrajectoryWithRew
from typing import List
from typing import Sequence
import imitation.policies.serialize as serial

scale_factor = 0.05
input_msg = Joy()
key = 0
curr_pos = PoseStamped()
is_gripping = 0
is_pose = 0

trajectories = []

position_x = 0.5
position_y = 0.5
position_z = 0.5
orientation_x = 0.5
orientation_y = 0.5
orientation_z = 0.5
orientation_w = 0.5

change_x = 0
change_y = 0
change_z = 0
change_x_ori = 0
change_y_ori = 0
change_z_ori = 0
change_w_ori = 0

def rewardFunction(trajectories : Trajectory) -> List[float]:
	location = trajectories.obs[0]
	object = np.array([0.50366, -0.00611, 0])
	goal = np.array([0.446417, 0.319033, 0.143698])
	rewards = []
	state = 0
	for action in trajectories.acts:
		action[:3] = np.clip(action[:3], -0.005, 0.005)
		if ((location[7] == 1) and np.allclose(object, location[:3], 0, 0.1)):
			object = object + action[:3]
			state = 1
		location[:7] = location[:7] + action[:7]
		if (location[2] < 0):
			location[2] = 0
			action[2] = 0
		if (action[7] != 1 or action[7] != 0):
			action[7] = round(action[7])
		location[7] = action[7]
		terminated = np.allclose(object[:2], goal[:2], 0, 0.1)
		terminated = terminated and not (location[7] == 1) and state == 1
		if terminated:
			reward = 1
			print("terminated")
		else:
			if state == 0:
				distance = np.linalg.norm(np.subtract(location[:3], goal), ord=1)
			else:
				distance = np.linalg.norm(np.subtract(location[:3], object), ord=1)
			if distance < 1:
				reward = (0.4 * (1 - distance)) + 0.4*state
			else:
				reward = 0
		rewards.append(reward)

	return rewards

def endCondition(trajectories : Sequence[TrajectoryWithRew]) -> bool:
	ended = False
	for traj in trajectories:
		for rew in traj.rews:
			if rew == 1:
				ended = True
	return ended

def move_gripper(gripperWidth):
	# Initialise action client to move gripper
	moveClient = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

	# Wait until server has started
	moveClient.wait_for_server()
	# Create a goal with passed-in width to the gripper
	goal = franka_gripper.msg.MoveGoal(width=gripperWidth, speed=1.0)

	moveClient.send_goal(goal)

	moveClient.wait_for_result()

	return moveClient.get_result()


def grasp_gripper(gripperWidth):
	# Initialise action client to move gripper
	moveClient = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

	# Wait until server has started
	moveClient.wait_for_server()
	# Create a goal with passed-in width to the gripper
	graspEpsilon = franka_gripper.msg.GraspEpsilon(inner=0.1, outer=0.1)
	goal = franka_gripper.msg.GraspGoal(width=gripperWidth, epsilon=graspEpsilon, speed=1.0, force=20)

	moveClient.send_goal(goal)

	# moveClient.wait_for_result()
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

	while (converting):
		if (lineNumber == len(instructions)):
			converting = False
			print("Run Completed\n")
		else:
			line = instructions[lineNumber]
			if (line == "Run:"):
				# Move to the first line of the run
				lineNumber = lineNumber + 1
			elif (line[0] == "p"):
				# Check desired pose position - there shouldn't be any individual orientation values for string
				desiredPose.pose.position.x = float(
					instructions[lineNumber + 1].translate({ord(i): None for i in 'x :'}))
				desiredPose.pose.position.y = float(
					instructions[lineNumber + 2].translate({ord(i): None for i in 'y :'}))
				desiredPose.pose.position.z = float(
					instructions[lineNumber + 3].translate({ord(i): None for i in 'z :'}))

				# Check desired pose orientation
				desiredPose.pose.orientation.x = float(
					instructions[lineNumber + 5].translate({ord(i): None for i in 'x :'}))
				desiredPose.pose.orientation.y = float(
					instructions[lineNumber + 6].translate({ord(i): None for i in 'y :'}))
				desiredPose.pose.orientation.z = float(
					instructions[lineNumber + 7].translate({ord(i): None for i in 'z :'}))
				desiredPose.pose.orientation.w = float(
					instructions[lineNumber + 8].translate({ord(i): None for i in 'w :'}))

				print("Moving To Pose\n")

				# Publish the pose then wait to ensure the system isn't queueing too badly
				posePub.publish(desiredPose)
				rate.sleep()

				# Move to the next line
				lineNumber = lineNumber + 9
			elif (line[0] == "G"):
				print("Closing gripper")
				grasp_gripper(0.01)
				lineNumber = lineNumber + 1
			elif (line[0] == "R"):
				print("Opening gripper")
				move_gripper(0.08)
				lineNumber = lineNumber + 1
			else:
				lineNumber = lineNumber + 1
				print("Huh, something weird in the file at line " + str(lineNumber))


def move_from_trajectories():
	# Reads an instruction file located at data/Run.txt and moves the robot
	runTrajectories = serialize.load("data/run.Dataset")
	desiredPose = PoseStamped()
	converting = True
	rate = rospy.Rate(4)
	print("Starting Read\n")
	print(len(runTrajectories))

	trajectory = runTrajectories[0]
	obs = np.asarray(trajectory.obs)
	print(obs)
	acts = trajectory.acts
	count = -1
	print(len(obs))
	lastGrip = 0.0
	for i in obs:
		# Check desired pose position - there shouldn't be any individual orientation values for string
		if count == -1:
			desiredPose.pose.position.x = i[0]
			desiredPose.pose.position.y = i[1]
			desiredPose.pose.position.z = i[2]

			# Check desired pose orientation
			desiredPose.pose.orientation.x = i[3]
			desiredPose.pose.orientation.y = i[4]
			desiredPose.pose.orientation.z = i[5]
			desiredPose.pose.orientation.w = i[6]

			move_gripper(0.08)

		else:
			desiredPose.pose.position.x = i[0] + acts[count][0]
			desiredPose.pose.position.y = i[1] + acts[count][1]
			desiredPose.pose.position.z = i[2] + acts[count][2]

			# Check desired pose orientation
			desiredPose.pose.orientation.x = i[3] + acts[count][3]
			desiredPose.pose.orientation.y = i[4] + acts[count][4]
			desiredPose.pose.orientation.z = i[5] + acts[count][5]
			desiredPose.pose.orientation.w = i[6] + acts[count][6]

			if ((acts[count][7] == 1.0) and (lastGrip == 0.0)):
				print("Closing gripper")
				grasp_gripper(0.01)
			elif ((acts[count][7] == 0.0) and (lastGrip == 1.0)):
				print("Opening gripper")
				move_gripper(0.08)
			lastGrip = i[7]
		count = count + 1
		# Publish the pose then wait to ensure the system isn't queueing too badly
		posePub.publish(desiredPose)
		if count == -1:
			rospy.sleep(4)
		rate.sleep()


def joystick_control(recording):

	global key, change_x, change_y, change_z, change_x_ori, change_y_ori, change_z_ori, change_w_ori, is_gripping, is_pose, curr_pos, key, scale_factor, input_msg, trajectories, currentTrajectory, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w
	running = True
	# Initialise Publisher and Subscriber
	poseSub = rospy.Subscriber('/cartesian_pose', PoseStamped, pose_callback)
	inputSub = rospy.Subscriber('/spacenav/joy', Joy, control_callback)

	# Open the record file
	# logFile = open("data/pickAndPlaceRuns.txt", "a")
	# logFile.write("Run\n")
	# logFile.close()
	if (recording):
		env = make_vec_env(
			'PandaSim-v0',
			rng=np.random.default_rng(),
			post_wrappers=[
				lambda env, _: RolloutInfoWrapper(env)
			],  # needed for computing rollouts later
		)
		datasetFile = Path("data/trajectories.Dataset")
		fileExists = datasetFile.exists()
		if fileExists:
			loadedTrajectories = serialize.load("data/trajectories.Dataset")
		env.reset()

		obs = []
		acts = []

	firstRun = True

	print("Launching Joystick Interface, press ctrl-c to end\n")
	try:
		while running:
			if key == 1:
				ctrl_msg = PoseStamped()
				rate = rospy.Rate(5)
				global scale_factor
				global curr_pos
				rotation_factor = 10
				changed = False

				if not rospy.is_shutdown():
					# orgAngleEuler = orgAngle.as_euler('xyz',degrees=True)
					# rot = Rotation.from_euler('xyz', [(input_msg.angular.x*5)+orgAngleEuler[0],(input_msg.angular.y*5)+orgAngleEuler[1],(input_msg.angular.x*5)+orgAngleEuler[2]], degrees=True)
					if firstRun:
						rate.sleep()
						# print(curr_pos)
						position_x = curr_pos.pose.position.x
						position_y = curr_pos.pose.position.y
						position_z = curr_pos.pose.position.z

						orientation_x = curr_pos.pose.orientation.x
						orientation_y = curr_pos.pose.orientation.y
						orientation_z = curr_pos.pose.orientation.z
						orientation_w = curr_pos.pose.orientation.w
						if (recording):
							obs.append(np.array([curr_pos.pose.position.x, curr_pos.pose.position.y,
												 curr_pos.pose.position.z, curr_pos.pose.orientation.x,
												 curr_pos.pose.orientation.y, curr_pos.pose.orientation.z,
												 curr_pos.pose.orientation.w, is_gripping]))
						firstRun = False

					if (abs(input_msg.axes[0]) >= 0.05):
						ctrl_msg.pose.position.x = curr_pos.pose.position.x + (input_msg.axes[0] * scale_factor)
						change_x = (input_msg.axes[0] * scale_factor)
						position_x = ctrl_msg.pose.position.x
						# print("X: " + str(position_x))
						changed = True
					else:
						ctrl_msg.pose.position.x = position_x
						change_x = 0
						changed = False or changed
					if (abs(input_msg.axes[1]) >= 0.05):
						ctrl_msg.pose.position.y = curr_pos.pose.position.y + (input_msg.axes[1] * scale_factor)
						change_y = (input_msg.axes[1] * scale_factor)
						position_y = ctrl_msg.pose.position.y
						# print("Y: " + str(position_y))
						changed = True
					else:
						ctrl_msg.pose.position.y = position_y
						change_y = 0
						changed = False or changed
					if (abs(input_msg.axes[2]) >= 0.05):
						ctrl_msg.pose.position.z = curr_pos.pose.position.z + (input_msg.axes[2] * scale_factor)
						change_z = (input_msg.axes[2] * scale_factor)
						position_z = ctrl_msg.pose.position.z
						# print("Z: " + str(position_z))
						changed = True
					else:
						ctrl_msg.pose.position.z = position_z
						change_z = 0
						changed = False or changed
					# print(curr_pos)
					if False:
						# if ((abs(input_msg.axes[3]) >= 0.1) or (abs(input_msg.axes[4]) >= 0.1)  or (abs(input_msg.axes[5]) >= 0.1)):
						orgAngle = Rotation.from_quat(
							[curr_pos.pose.orientation.x, curr_pos.pose.orientation.y, curr_pos.pose.orientation.z,
							 curr_pos.pose.orientation.w])
						rot = Rotation.from_euler('xyz', [(input_msg.axes[3] * rotation_factor),
														  (-input_msg.axes[4] * rotation_factor),
														  (-input_msg.axes[5] * rotation_factor)], degrees=True)
						combined = orgAngle * rot
						quaternion = combined.as_quat()
						# print(quaternion)
						ctrl_msg.pose.orientation.x = quaternion[0]
						ctrl_msg.pose.orientation.y = quaternion[1]
						ctrl_msg.pose.orientation.z = quaternion[2]
						ctrl_msg.pose.orientation.w = quaternion[3]
						orientation_x = quaternion[0]
						orientation_y = quaternion[1]
						orientation_z = quaternion[2]
						orientation_w = quaternion[3]
						print(quaternion)
					else:
						ctrl_msg.pose.orientation.x = orientation_x
						ctrl_msg.pose.orientation.y = orientation_y
						ctrl_msg.pose.orientation.z = orientation_z
						ctrl_msg.pose.orientation.w = orientation_w

					if (input_msg.buttons[0] == 1):
						result = grasp_gripper(0)
						is_gripping = 1
						changed = True
					elif (input_msg.buttons[1] == 1):
						result = move_gripper(0.08)
						is_gripping = 0
						changed = True

					if (changed):
						if (recording):
							obs.append(np.array([curr_pos.pose.position.x, curr_pos.pose.position.y,
												 curr_pos.pose.position.z, curr_pos.pose.orientation.x,
												 curr_pos.pose.orientation.y, curr_pos.pose.orientation.z,
												 curr_pos.pose.orientation.w, is_gripping]))
							acts.append(np.array([change_x, change_y, change_z, change_x_ori, change_y_ori,
												  change_z_ori, change_w_ori, is_gripping]))
						# print(currentTrajectory)
						# print(observations)
						# print(actions)
						# print (ctrl_msg)
						posePub.publish(ctrl_msg)
					# logFile = open("data/pickAndPlaceRuns.txt", "a")
					# logFile.write(str(ctrl_msg.pose)+"\n")
					# logFile.close()
					rate.sleep()
				else:
					break
				key = 0
	except KeyboardInterrupt:
		running = False
		key = 0
		pass
	print("Done")
	if (recording):
		obs.append(np.array([curr_pos.pose.position.x, curr_pos.pose.position.y,
								 curr_pos.pose.position.z, curr_pos.pose.orientation.x,
								 curr_pos.pose.orientation.y, curr_pos.pose.orientation.z,
								 curr_pos.pose.orientation.w, is_gripping]))
		acts.append(np.array([change_x, change_y, change_z, change_x_ori, change_y_ori,
							change_z_ori, change_w_ori, is_gripping]))
		print("Length:")
		print(len(obs))
		print(len(acts))
		currentTrajectory = Trajectory(np.array(obs),np.array(acts),None,True)
		#rews = rewardFunction(currentTrajectory)
		#currentTrajectory = TrajectoryWithRew(np.array(obs),np.array(acts),None,True,np.array(rews))
		trajectories.append(currentTrajectory)
		if fileExists:
			serialize.save("data/trajectories.Dataset", list(loadedTrajectories) + trajectories)
		else:
			serialize.save("data/trajectories.Dataset", trajectories)


def learnBC():
	rng = np.random.default_rng(0)
	#env = gym.make('PandaSim-v0')
	env = make_vec_env(
		'PandaSim-v0',
		rng=np.random.default_rng(),
		post_wrappers=[
			lambda env, _: RolloutInfoWrapper(env)
		],  # needed for computing rollouts later
	)
	trajectories = serialize.load("data/trajectories.Dataset")

	print(trajectories)
	transitions = rollout.flatten_trajectories(trajectories)

	# print("Training a expert.")

	bc_trainer = bc.BC(
		observation_space=env.observation_space,
		action_space=env.action_space,
		demonstrations=trajectories,
		rng=rng,
		batch_size=64,
	)
	bc_trainer.train(n_epochs=40,progress_bar=True)
	print("Trained BC")
	learner = PPO(
		policy="MlpPolicy",
		env=env,
		seed=0,
		batch_size=64,
		ent_coef=0.0,
		learning_rate=0.0003,
		n_epochs=10,
		n_steps=64,
	)
	print("Created Agent")
	#learner.learn(100_000)  # Note: change this to 100_000 to train a decent expert.

	reward_net = BasicRewardNet(
		observation_space=env.observation_space,
		action_space=env.action_space,
		normalize_input_layer=RunningNorm,
	)
	gail_trainer = GAIL(
		demonstrations=trajectories,
		demo_batch_size=50,
		gen_replay_buffer_capacity=512,
		n_disc_updates_per_round=8,
		venv=env,
		gen_algo=learner,
		reward_net=reward_net,
	)
	print("Trained GAIL")



	print("Observations")
	print(bc_trainer.observation_space)
	print("Actions")
	print(bc_trainer.action_space)
	print("Trajectories")
	print(len(trajectories))
	print(trajectories[0])
	#bc_trainer.set_demonstrations(transitions)
	print("Training")
	#gail_trainer.train(10000)
	print("Autobots, Roll Out!")
	#serial.save_stable_model("data/",bc_trainer.policy,"model.zip")
	isCompletable = 1
	counter = 0
	print("Policy:")
	print(bc_trainer.policy.)

	while(isCompletable):
		output_trajectories = rollout.rollout(
			bc_trainer.policy,
			env,
			sample_until=rollout.make_min_episodes(20),
			rng=np.random.default_rng(),
			unwrap=False,
		)
		trajectory = output_trajectories[0]
		obs = np.asarray(trajectory.obs)
		counter+=1
		if len(obs) != 5001:
			isCompletable = 0
		elif counter == 5:
			isCompletable = 0
			print("Can't generate a good trajectory from this policy")

	#print(output_trajectories)

	print("Saving!")
	serialize.save("data/run.Dataset", output_trajectories)

def learnDagger(limit):
	for i in range(limit):
		if(i%2 == 0):
			learnBC()
			move_from_trajectories()
		else:
			move_to_bases('s')
			joystick_control(1)

def move_to_bases(base):
	ctrl_msg = PoseStamped()
	ctrl_msg.pose.orientation.x = 0.999442
	ctrl_msg.pose.orientation.y = 0.0244632
	ctrl_msg.pose.orientation.z = 0.0019144
	ctrl_msg.pose.orientation.w = 0.031580
	if (base == 'b'):
		#0.489728,-0.005413,0.258339,0.999442,0.0244632,0.0019144,0.031580,0
		ctrl_msg.pose.position.x = 0.489728
		ctrl_msg.pose.position.y = -0.005413
		ctrl_msg.pose.position.z =0.258339
		messedup = 0
	elif (base == 'o'):
		#0.50366, -0.00611, 0
		ctrl_msg.pose.position.x = 0.50366
		ctrl_msg.pose.position.y = -0.00611
		ctrl_msg.pose.position.z =0
		messedup = 0
	elif (base == 'g'):
		#0.446417,0.319033,0.143698
		ctrl_msg.pose.position.x = 0.446417
		ctrl_msg.pose.position.y = 0.319033
		ctrl_msg.pose.position.z =0.143698
		messedup = 0
	elif (base == 's'):
		randomiser = np.random.random(size=2)
		randomiser[0] -= 0.5
		randomiser[1] -= 0.5
		randomiser[0] = randomiser[0] * 0.15
		randomiser[1] = randomiser[0] * 0.15
		#0.489728,-0.005413,0.258339,0.999442,0.0244632,0.0019144,0.031580,0
		ctrl_msg.pose.position.x = 0.489728 + randomiser[0]
		ctrl_msg.pose.position.y = -0.005413 +randomiser[1]
		ctrl_msg.pose.position.z =0.258339
		messedup = 0
	else:
		print("Asked to move to a base that doesn't exist\n")
		messedup = 1

	if not messedup:
		posePub.publish(ctrl_msg)
		rospy.sleep(2)

def main():
	running = True
	rospy.init_node('Simulator_Interface', anonymous=True, disable_signals=True)
	while (running):
		check = input("Press r to run from learned trajectories, n to record new run, t to teleoperate without recording, l to learn, b to move to base pose, g to move to goal pose, o to move to object position, or s to move to randomised start position\n")
		if (check == 'r'):
			move_from_trajectories()
		elif (check == 'n'):
			#move_to_bases('s')
			joystick_control(1)
		elif (check == 't'):
			joystick_control(0)
		elif (check == 'l'):
			learnBC()
		elif (check == 'b'):
			move_to_bases(check)
		elif (check == 'o'):
			move_to_bases(check)
		elif (check == 'g'):
			move_to_bases(check)
		elif (check == 's'):
			move_to_bases(check)
		else:
			print("Make sure it's t, n, b or l lower case, nothing else, please and thank you\n")


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
