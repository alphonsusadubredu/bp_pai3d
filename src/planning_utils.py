import numpy as np  
import time
import pybullet_planning as pyplan

class utils:
	def __init__(self, robot, world):
		self.robot = robot
		self.world = world
		self.go_to_pile()

	def collision_score_traj_obst(self, traj, pose_max, armname='right_arm', obstacles=[]):
		'''
		1. score traj by summing up each waypoints distance to nearest obstacle
		2. augment to score how close it reaches pose_max 
		'''
		collision_cost = self.robot.get_traj_obst_cost(traj, armname, obstacles=obstacles)
		cost_to_goal = self.robot.get_traj_goal_cost(traj, pose_max, armname)

		score = 1/(np.exp(cost_to_goal*100))# 0.00001*collision_cost + 1/(cost_to_goal*10000)
		return score


	def collision_score_pose_obst(self, pose, traj_max, armname='right_arm'):
		'''
		1. score is how close pose gets to traj_max's end pose while avoiding  collisions
		'''
		cost_to_goal = self.robot.get_traj_goal_cost(traj_max, pose, armname)
		score =  1/(np.exp(cost_to_goal*100))
		return score


	def grasp_score_grasp(self, grasp):
		'''
		Defines stable grasp poses(transforms between the handframe and object frame)
		score_grasp scores how stable a grasp is. 
		'''
		score = self.robot.score_grasp(grasp)
		return score 

	def stable_score_stable(self,placement):
		score = self.robot.score_stable(placement)
		return score


	def kin_score_conf(self, conf, pmax, gmax, armname='right_arm'):
		score = self.robot.score_kin(conf, pmax, gmax)
		return score

	def kin_score_poses(self,cmax, pose,  gmax, armname='right_arm'):
		score = self.robot.score_kin(cmax, pose, gmax)
		return score

	def kin_score_grasps(self, cmax, pmax, grasp,  armname='right_arm'):
		score = self.robot.score_kin(cmax, pmax, grasp)
		return score

	def se2_traj_score(self, traj, pose):
		score = np.linalg.norm(np.array(traj[-1])-np.array(pose))
		return score

	def se2_pose_score(self, traj, pose):
		score = np.linalg.norm(np.array(traj[-1])-np.array(pose))
		return score

	def plan_arm_trajectory_to(self, pose):
		traj = self.robot.plan_arm_motion(pose, 'right_arm')
		return traj

	def compute_generic_grasp(self, pose):
		grasp = self.robot.get_generic_top_grasp(pose)
		return grasp

	def compute_ik_to_pose(self, pose):
		conf = self.robot.compute_ik_to_pose(pose[0],pose[1], 'right_arm')
		return conf 

	def plan_se2_motion_to(self, pose):
		traj = self.robot.plan_to_pose(pose)
		return traj

	def get_prior_belief(self, num_particles, targets):
		prior = self.world.get_prior_belief(num_particles, targets)
		return prior

	def go_to_pile(self):
		self.robot.plan_and_drive_to_pose(self.world.pick_base_pose, self.world.base_limits,obstacles=[self.world.main_table]) 
		time.sleep(2)
		print('at pile')

	def teleport_to_tray(self):
		self.init_pose = pyplan.get_base_values(self.robot.id)
		potential = [self.world.place_base_pose, self.world.wash_station, self.world.stove_station]
		dists = [0.0,0.0,0.0]
		current_pose = pyplan.get_base_values(self.robot.id)
		for i in range(3):
			dists[i] = np.linalg.norm((np.array(current_pose)[:-2] - np.array(potential[i])[:-2]))
		ind = np.argmin(dists)
		pyplan.set_base_values(self.robot.id, potential[ind])

	def get_tray_base_pose(self):
		potential = [self.world.place_base_pose, self.world.wash_station, self.world.stove_station]
		dists = [0.0,0.0,0.0]
		current_pose = pyplan.get_base_values(self.robot.id)
		for i in range(3):
			dists[i] = np.linalg.norm((np.array(current_pose)[:-2] - np.array(potential[i])[:-2]))
		ind = np.argmin(dists)
		return potential[ind]

	def teleport_to(self, place):
		self.init_pose = pyplan.get_base_values(self.robot.id)
		places = {'wash-station':self.world.wash_station,'stove-station':self.world.stove_station, 'pile-station':self.world.place_base_pose}
		pose = places[place]
		pyplan.set_base_values(self.robot.id, pose)

	def teleport_back(self):
		pyplan.set_base_values(self.robot.id, self.init_pose)