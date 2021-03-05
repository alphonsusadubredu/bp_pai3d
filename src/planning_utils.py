import numpy as np  
import time
import pybullet_planning as pyplan

class utils:
	def __init__(self, robot, world):
		self.robot = robot
		self.world = world
		self.drawers = ['top-left', 'top-right', 'middle']
		# self.go_to_pile()

	def choose_drawer(self):
		choice = np.random.choice(self.drawers)
		print('chosen ',choice)
		return choice

	def update_drawer_belief(self, notin):
		self.drawers.remove(notin)

	def collision_score_traj_obst(self, traj, pose_max, armname='right_arm', obstacles=[]):
		'''
		1. score traj by summing up each waypoints distance to nearest obstacle
		2. augment to score how close it reaches pose_max 
		'''
		# pose_max = pyplan.get_pose(self.world.get_id('pear'))
		collision_cost = self.robot.get_traj_obst_cost(traj, armname, obstacles=obstacles)
		cost_to_goal = self.robot.get_traj_goal_cost(traj, pose_max, armname)

		score = 1/(np.exp(cost_to_goal*10)) + collision_cost 
		return score


	def collision_score_pose_obst(self, pose, traj_max, armname='right_arm'):
		'''
		1. score is how close pose gets to traj_max's end pose while avoiding  collisions
		'''
		cost_to_goal = self.robot.get_traj_goal_cost(traj_max, pose, armname)
		score =  1/(np.exp(cost_to_goal*10))
		return score


	def grasph_score_grasp(self, grasp):
		'''
		Defines stable grasp poses(transforms between the handframe and object frame)
		score_grasp scores how stable a grasp is. 
		'''
		score = self.robot.score_grasp(grasp)
		return score 

	def grasp_score_grasp(self, conf, grasp, pose, pose2, conf2):
		opt = self.world.opt_pour
		score = 1/(0.1+self.robot.score_graspc(conf, grasp, pose, pose2, conf2, opt))
		return score 

	def grasp_score_conf1(self, conf, grasp, pose, pose2, conf2):
		sigma = self.world.joint_noise
		num_samples = 10
		confs_bunch = []
		for joint in conf:
			joint_vals = np.random.normal(joint, sigma, size=num_samples)
			confs_bunch.append(joint_vals)
		confs = [[js[i] for js in confs_bunch] for i in range(len(conf))]

		total_score = 0
		gaussian_probs = []
		graspscores = []
		for cf in confs:
			gp = self.get_cf_prob(conf, cf)
			graspscore = self.robot.score_conf_graspc(conf, grasp, pose)
			gaussian_probs.append(gp)
			graspscores.append(graspscore)
		gaussian_probs = np.array(gaussian_probs)/np.sum(gaussian_probs)
		total_score = np.dot(gaussian_probs, np.array(graspscores))
		return total_score 
		

	def grasp_score_pose1(self, conf, grasp, pose, pose2, conf2):
		score = self.kin_score_poses(conf, pose, grasp)
		return score 

	def grasp_score_conf2(self, conf, grasp, pose, pose2, conf2):
		sigma = self.world.joint_noise
		num_samples = 10
		confs_bunch = []
		for joint in conf:
			joint_vals = np.random.normal(joint, sigma, size=num_samples)
			confs_bunch.append(joint_vals)
		confs = [[js[i] for js in confs_bunch] for i in range(len(conf))]

		total_score = 0
		gaussian_probs = []
		kinscores = []
		for cf in confs:
			gp = self.get_cf_prob(conf, cf)
			kinscore = self.robot.score_conf_graspc(conf2, grasp, pose2)
			gaussian_probs.append(gp)
			kinscores.append(kinscore)
		gaussian_probs = np.array(gaussian_probs)/np.sum(gaussian_probs)
		total_score = np.dot(gaussian_probs, np.array(kinscores))
		return total_score 
		

	def grasp_score_pose2(self, conf, grasp, pose, pose2, conf2):
		score = self.kin_score_poses(conf2, pose2, grasp)
		return score 


	def stable_score_stable(self,placement):
		score = self.robot.score_stable(placement)
		return score


	def kin_score_conf(self, conf, pmax, gmax, armname='right_arm'):
		sigma = self.world.joint_noise
		num_samples = 10
		confs_bunch = []
		for joint in conf:
			joint_vals = np.random.normal(joint, sigma, size=num_samples)
			confs_bunch.append(joint_vals)
		confs = [[js[i] for js in confs_bunch] for i in range(len(conf))]

		total_score = 0
		gaussian_probs = []
		kinscores = []
		for cf in confs:
			gp = self.get_cf_prob(conf, cf)
			kinscore = self.robot.score_kin(cf, pmax, gmax)
			gaussian_probs.append(gp)
			kinscores.append(kinscore)
		gaussian_probs = np.array(gaussian_probs)/np.sum(gaussian_probs)
		total_score = np.dot(gaussian_probs, np.array(kinscores))
		return total_score 

	def get_cf_prob(self, mean, cf):
		score = np.linalg.norm((np.array(mean) - np.array(cf)))
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

	def get_prior_belief(self, num_particles, targets,sigma=0.001):
		prior = self.world.get_prior_belief(num_particles, targets,sigma=sigma)
		return prior

	def get_conf_noise(self, conf):
		sigma = self.world.joint_noise
		num_samples = 10
		confs_bunch = []
		for joint in conf:
			joint_vals = np.random.normal(joint, sigma, size=num_samples)
			confs_bunch.append(joint_vals)
		confs = [[js[i] for js in confs_bunch] for i in range(len(conf))]
		return confs 



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
		places = {'wash-station':self.world.sink_base_pose,'stove-station':self.world.stove_base_pose, 'dining-station':self.world.plate_base_pose, 'mug-station':self.world.mug_base_pose}
		pose = places[place]
		pyplan.set_base_values(self.robot.id, pose)


	def teleport_back(self):
		pyplan.set_base_values(self.robot.id, self.init_pose)


	def reweight_resample_particles(self, particles, obs):
		new_parts = []
		resampled = []
		for part,wt in particles:
			pt = part[0] 
			dist = np.linalg.norm((np.array(pt)[:2] - np.array(obs)[:2]))
			new_parts.append((pt, 1/dist))
		indices = [i for i in range(len(new_parts))]
		weights = [b[1] for b in new_parts]
		weights = np.array(weights)/np.sum(weights)
		sampled_indices = np.random.choice(indices, size=len(weights), p=weights )
		for ind in sampled_indices:
			resampled.append((particles[ind][0], weights[ind]))
		return resampled


	def get_max(self, particles):
		weights = [w[1] for w in particles]
		maxind = np.argmax(weights)
		maxpart = particles[maxind]
		return maxpart[0]


	def get_weighted_sum(self, particles):
		xs=[]; ys=[]; zs=[]; ws = [];
		for pt in particles:
			# print(pt)
			xs.append(pt[0][0][0])
			ys.append(pt[0][0][1])
			zs.append(pt[0][0][2])
			ws.append(pt[1])
		x = np.dot(np.array(xs), np.array(ws)/np.sum(ws))
		y = np.dot(np.array(ys), np.array(ws)/np.sum(ws))
		z = np.dot(np.array(zs), np.array(ws)/np.sum(ws))
		return (x,y,z)

	def get_weighted_sum_grasp(self, particles):
		xs=[]; ys=[]; zs=[]; ws = []; ors = []
		for pt in particles:
			# print(pt)
			xs.append(pt[0][0][0])
			ys.append(pt[0][0][1])
			zs.append(pt[0][0][2])
			ors.append(pt[0][1])
			ws.append(pt[1])
		x = np.dot(np.array(xs), np.array(ws)/np.sum(ws))
		y = np.dot(np.array(ys), np.array(ws)/np.sum(ws))
		z = np.dot(np.array(zs), np.array(ws)/np.sum(ws))
		rr = [i for i in range(len(ors))]
		orr = ors[np.random.choice(rr)]

		return ((x,y,z),orr)


	def particle_filter(self, buff_plan):
		for action in buff_plan:
			for var in action.variables:
				if var.type == 'Pose':
					particles = var.belief_update()  
					if action.obj[0] == '':
						obs = self.world.get_noisy_pose('pear')
					else:
						obs = self.world.get_noisy_pose(action.obj[0])
					particles = self.reweight_resample_particles(particles, obs)
					pose = self.get_weighted_sum(particles)
					action.pfs['Pose'] = pose
				elif var.type == 'Grasp':
					particles = var.belief_update()
					obs = self.world.get_noisy_grasp(self.world.opt_pour)
					particles = self.reweight_resample_particles(particles, obs)
					grasp = self.get_weighted_sum_grasp(particles)
					action.pfs['Grasp'] = grasp 
		return buff_plan