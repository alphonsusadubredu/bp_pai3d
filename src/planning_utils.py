import numpy as np  

class utils:
	def __init__(self, robot):
		self.robot = robot

	def collision_score_traj_obst(self, traj, pose_max, armname, obstacles):
		'''
		1. score traj by summing up each waypoints distance to nearest obstacle
		2. augment to score how close it reaches pose_max 
		'''
		collision_cost = self.robot.get_traj_obst_cost(traj, armname, obstacles=obstacles)
		cost_to_goal = self.robot.get_traj_goal_cost(traj, pose_max, armname)

		score = 0.0001*collision_cost + cost_to_goal
		return score


	def collision_score_pose_obst(self, pose, traj_max, armname):
		'''
		1. score is how close pose gets to traj_max's end pose while avoiding  collisions
		'''
		cost_to_goal = self.robot.get_traj_goal_cost(traj_max, pose, armname)
		return cost_to_goal


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


	def kin_score_conf(self, conf, pmax, gmax, armname):
		score = self.robot.score_kin(conf, pmax, gmax)
		return score

	def kin_score_poses(self,cmax, pose,  gmax, armname):
		score = self.robot.score_kin(cmax, pose, gmax)
		return score

	def kin_score_grasps(self, cmax, pmax, grasp,  armname):
		score = self.robot.score_kin(cmax, pmax, grasp)
		return score