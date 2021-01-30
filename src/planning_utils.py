import numpy as np  

class utils:
	def __init__(self, robot, world):
		self.robot = robot
		self.world = world

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