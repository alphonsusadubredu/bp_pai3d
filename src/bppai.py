import numpy as np 
# import planning_utils as pu
from fd import Fast_Downward
import copy 

class Action:
	def __init__(self, name, obj=None, constraints=[], variables=[]):
		self.name = name
		self.obj = obj
		self.obj_id = None
		self.constraints = constraints
		self.variables = variables
		self.const_ids=[]
		self.maps={}

	def get_map_for_variables(self):
		for var in self.variables:
			self.maps[var.type] = var.get_map()

class Variable_node:
	def __init__(self, name, typename,actionname, actiontarget, pu):
		self.name = name
		self.type = typename
		self.action_name = actionname
		self.action_target = actiontarget
		self.num_particles = 50
		self.weighted_particles={} 
		self.pu = pu

	def send_msg_to_constraint(self, constraint):
		all_particles = [] 
		for c in self.weighted_particles:
			# if c is not constraint:
			all_particles+=self.weighted_particles[c]
		if len(all_particles) == 0:
			# print('using prior')
			all_particles = self.prior_particles
		wts = [pw[1] for pw in all_particles]
		pts = [pw[0] for pw in all_particles]
		wts = wts/np.sum(wts)
		msg = [(pt,wt) for pt, wt in zip(pts, wts)] 
		return msg 


	def receive_msg_from_constraint(self, constraint, msg): 
		self.weighted_particles[constraint] = msg


	def belief_update(self):
		all_particles=[]
		for c in self.weighted_particles: 
			if self.weighted_particles[c] is not None:
				all_particles+=self.weighted_particles[c]
		if len(all_particles) == 0:  
			# print('using prior')
			all_particles = copy.deepcopy(self.prior_particles) 
		wts = [pw[1] for pw in all_particles]
		pts = [pw[0] for pw in all_particles]
		wts = wts/np.sum(wts)
		belief = [(pt,wt) for pt, wt in zip(pts, wts)]
		return belief 


	def get_sample_from_belief(self):
		belief = self.belief_update()
		particles = []
		indices = [i for i in range(len(belief))]
		weights = [b[1] for b in belief]
		sampled_indices = np.random.choice(indices, size=len(weights), p=weights )
		for ind in sampled_indices:
			particles.append(belief[ind])
		return particles 

	def get_map(self):
		belief = self.belief_update()
		weights = [w[1] for w in belief]
		maxind = np.argmax(weights)
		maxpart = belief[maxind] 
		return maxpart[0]


	def initialize_with_prior(self, prior): 
		if self.action_target != '': 
			self.prior_pose_particles = prior[self.action_target]
			self.prior_particles = prior[self.action_target]
			
			if self.action_name == 'put-on-tray' or self.action_name == 'carry-tray':
				# print('tray initing')
				self.prior_pose_particles = prior['tray']
				if self.type == "Pose":
					self.prior_particles = prior['tray']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to_tray()
					for (pose, wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.pu.teleport_back()
					self.prior_particles = pts 
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to_tray()
					for (pose,wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.pu.teleport_back()
					self.prior_particles = pts 
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to_tray()
					for (pose,wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
					self.pu.teleport_back()
					self.prior_particles = pts



			elif self.action_name == 'pick' or self.action_name == 'wash' or self.action_name == 'cook':
				# print(self.action_name, 'initing')
				if self.type == "Pose":
					self.prior_particles = prior[self.action_target]
				elif self.type == "Trajectory":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
				elif self.type == "Grasp":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
				elif self.type == "Configuration":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
					self.prior_particles = pts 
		else:
			if self.action_name == 'go-to-wash-station':
				self.prior_pose_particles = prior['wash-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['wash-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts

			elif self.action_name == 'go-to-stove':
				self.prior_pose_particles = prior['stove-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['stove-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts  


	def initialize_with_prior_old(self, prior):
		if self.action_target != '': 
			self.prior_pose_particles = prior[self.action_target]
			self.prior_particles = self.prior_pose_particles
			
			if self.action_name == 'put-on-tray' or self.action_name == 'carry-tray':
				self.prior_pose_particles = prior['tray']

			if self.type == "Pose":
				self.prior_particles = self.prior_pose_particles
			elif self.type == "Trajectory":
				pts = []
				for (pose,wt) in self.prior_pose_particles:
					grasp = self.pu.compute_generic_grasp(pose)
					traj = self.pu.plan_arm_trajectory_to(grasp) 
					pts.append((traj,wt))
				self.prior_particles = pts 
			elif self.type == "Grasp":
				pts = []
				for (pose,wt) in self.prior_pose_particles:
					grasp = self.pu.compute_generic_grasp(pose)
					pts.append((grasp,wt))
				self.prior_particles = pts 
			elif self.type == "Configuration":
				pts = []
				for (pose,wt) in self.prior_pose_particles:
					grasp = self.pu.compute_generic_grasp(pose)
					conf = self.pu.compute_ik_to_pose(grasp)
					pts.append((conf,wt))
				self.prior_particles = pts 
		else:
			if self.action_name == 'go-to-wash-station':
				self.prior_pose_particles = prior['wash-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['wash-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts

			elif self.action_name == 'go-to-stove':
				self.prior_pose_particles = prior['stove-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['stove-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					for (pose,wt) in self.prior_pose_particles:
						traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts  








class Constraint_node:
	def __init__(self, name, variables, constraint_function):
		self.constraint_function = constraint_function
		self.variables = variables
		self.name = name
		self.variable_msgs = {}

	def receive_msg_from_variable(self, variabletype, msg):
		self.variable_msgs[variabletype] = msg  

	def send_msg_to_variable(self, sampled_belief, variablename, variabletype):
		if self.name == 'CFree':
			msg = None
			if variabletype == 'Trajectory': 
				msg = self.constraint_function(sampled_belief, self.variable_msgs['Pose'], target=variabletype)
			elif variabletype == 'Pose':
				msg = self.constraint_function(self.variable_msgs['Trajectory'],sampled_belief, target=variabletype)
			if msg is None:
				print(variabletype)
			return msg

		elif self.name == 'Kin':
			msg = None
			if variabletype == 'Grasp':
				msg = self.constraint_function(sampled_belief, self.variable_msgs['Configuration'], self.variable_msgs['Pose'],target=variabletype)
			elif variabletype == 'Configuration':
				msg = self.constraint_function(self.variable_msgs['Grasp'], sampled_belief, self.variable_msgs['Pose'],target=variabletype)
			elif variabletype == 'Pose':
				msg = self.constraint_function(self.variable_msgs['Grasp'], self.variable_msgs['Configuration'], sampled_belief, target=variabletype)
			if msg is None:
				print(variabletype)
			return msg 

		elif self.name == 'Grasp':
			msg = self.constraint_function(sampled_belief, None, None)
			if msg is None:
				print(variabletype)
			return msg 

		elif self.name == 'Stable':
			msg = self.constraint_function(sampled_belief, None, None)
			if msg is None:
				print(variabletype)
			return msg 

		elif self.name == 'CFree-move':
			if variabletype == 'SE2-Trajectory':
				msg = self.constraint_function(sampled_belief, self.variable_msgs['SE2-Pose'],target=variabletype)
			elif variabletype == 'SE2-Pose':
				msg = self.constraint_function(self.variable_msgs['SE2-Trajectory'], sampled_belief, target=variabletype)
			return msg

  


class HCSP: 
	constrained_actions = None
	factor_graph = {}
	skeleton = None


class Plan_skeleton:
	def __init__(self, instructions,pu=None):
		self.actions=[]
		self.instructions=instructions
		self.pu = pu
		self.constraints = {'CFree':[('T','Trajectory'),('p', 'Pose')],
							'Kin':[('p', 'Pose'),('g','Grasp'),('q','Configuration')],
							'Stable':[('p', 'Pose')],
							'Grasp':[('g', 'Grasp')],
							'CFree-move':[('M','SE2-Trajectory'),('MP','SE2-Pose')]}
		self.cfuncs = {'CFree': self.collision_free_func,
					   'Kin': self.kin_func,
					   'Stable':self.stable_func,
					   'Grasp':self.grasp_func,
					   'CFree-move':self.se2_func}
		self.domain_path = 'pddl/cook_domain.pddl' 
		self.ingredients = ['pear']#,'strawberry', 'meatcan']
		self.constraints_of_actions = {
			 'pick':['CFree', 'Kin', 'Grasp'],
			 'put-on-tray':['CFree','Kin', 'Stable'],
			 'carry-tray':['CFree', 'Kin', 'Grasp'],
			 'wash':['CFree','Kin'],
			 'cook':['CFree','Kin', 'Stable'],
			 'go-to-wash-station':['CFree-move'],
			 'go-to-stove':['CFree-move']
			}
		self.instruction_index = {}


	def get_skeleton(self):
		#forward search to densify instructions
		prev_state = ' (robot-at-foodstuff-station) (handempty) (in-pile pear) (in-pile strawberry) (in-pile meatcan) '
		skeleton = []
		ind = 0
		for i,instruction in enumerate(self.instructions):
			goal_state = self.infer_goal_state(instruction)
			plan = self.plan_from(prev_state, goal_state) 
			print('\n',plan, '\n')
			if plan is not None:
				ind += len(plan)
				self.instruction_index[instruction]=(ind,i)
				skeleton+=plan 
			prev_state = goal_state + ' (handempty)'
		self.skeleton = skeleton 
		return skeleton

		 
	def get_constraint_sketch(self, skeleton):
		pick = Action('pick','mug', ['CFree', 'Kin'], ['q','p','g','T'])
		place = Action('place','mug', ['CFree', 'Kin'], ['q','p','g','T'])
		pass

	def infer_goal_state(self, action):
		state = ''
		if action[0] == 'get': 
			state = ' (in-tray '+action[1]+')' 

		elif action[0] == 'wash': 
			state = ' (clean '+action[1]+')' 

		elif action[0] == 'cook': 
			state = ' (cooked '+action[1]+')'

		return state

	def plan_from(self, init_state, final_state): 
		problem_path = self.form_symbolic_planning_problem(init_state, final_state)
		fastD = Fast_Downward()
		plan = fastD.plan(self.domain_path, problem_path) 
		return plan 

	def form_symbolic_planning_problem(self, init_state, goal_state):
		definition = "(define (problem ROBOT-COOK) \n(:domain COOK) \n (:objects "
		for obj in self.ingredients:
			definition += obj+" "
		definition += "- ingredient)\n"

		init_state = '(:init '+init_state+') \n'
		goal_state = '(:goal (and '+goal_state+'))) \n'
		problem = definition + init_state + goal_state
		problem_path = 'pddl/problem.pddl'
		f = open(problem_path, 'w')
		f.write(problem)
		f.close()
		return problem_path


	def assign_constraints_to_actions(self, skeleton):
		action_skeleton = []
		for action in skeleton:
			act = Action(action[0],action[1])
			constraints = self.constraints_of_actions[action[0]]
			cc=[]
			var=[]
			for c in constraints:
				func = self.cfuncs[c]
				constobj = Constraint_node(c, self.constraints[c], func)
				cc.append(constobj)
				# var+=c.variables
			act.constraints=cc
			# act.variables = var
			action_skeleton.append(act)
		return action_skeleton


	# def build_hcsp(self):
	# 	hcsp = HCSP()
	# 	cons = []
	# 	for c in self.constraints: #change self.constraints to something else
	# 		func = self.cfuncs[c]
	# 		constraint = Constraint_node(c, self.constraints[c], func)
	# 		variables = [Variable_node(n[0],n[1]) for n in self.constraints[c]]
	# 		hcsp.factor_graph[c] = [constraint, (variables)] 
	# 	return hcsp


	def build_hcsp_from_constrained_action_skeleton(self, const_actions):
		hcsp = HCSP()
		hcsp.constrained_actions = const_actions
		index = 0
		for act in hcsp.constrained_actions:  
			for const in act.constraints: 
				var_objs = [Variable_node(n[0], n[1], act.name, act.obj,self.pu) for n in const.variables]
				hcsp.factor_graph[act.name+act.obj+const.name+str(index)] = [const, var_objs]
				act.const_ids.append(act.name+act.obj+const.name+str(index)) 
				index +=1
		return hcsp


	def build_hcsp(self):
		# skeleton = self.get_skeleton()
		constrained_actions = self.assign_constraints_to_actions(self.skeleton)
		hcsp = self.build_hcsp_from_constrained_action_skeleton(constrained_actions)
		hcsp.skeleton = self.skeleton 
		return hcsp


	def print_hcsp(self, hcsp):
		print('\n Skeleton: ', hcsp.skeleton)
		print('\nnumber of discrete actions: ',len(hcsp.constrained_actions))
		num_constraints_in_actions = 0
		for a in hcsp.constrained_actions:
			num_constraints_in_actions+=len(a.constraints)
		print('total number of constraints in actions: ',num_constraints_in_actions)
		print('\nnumber of actions in 1: ',len(hcsp.constrained_actions[0].constraints))
		for c in hcsp.factor_graph:
			print(c)
			print([v.name for v in hcsp.factor_graph[c][1]])
			print('______________________')


	def collision_free_func(self, trajs, poses, target):
		if target == 'Trajectory':
			traj_weights=[] 
			pose_max = self.get_best(poses)
			only_traj = [t[0] for t in trajs]
			for traj in only_traj:
				wt = self.pu.collision_score_traj_obst(traj, pose_max)
				traj_weights.append(wt)
			traj_weights = traj_weights/np.sum(traj_weights)
			msg = [(pt, wt) for pt,wt in zip(only_traj, traj_weights)]
			return msg

		elif target == 'Pose':
			pose_weights=[] 
			traj_max = self.get_best(trajs)
			only_pose = [p[0] for p in poses]
			for pose in only_pose:
				wt = self.pu.collision_score_pose_obst(pose, traj_max)
				pose_weights.append(wt)
			pose_weights = pose_weights/np.sum(pose_weights)
			msg = [(pt,wt) for pt, wt in zip(only_pose, pose_weights)]
			return msg


	def grasp_func(self, grasps, blah=None, bloh=None):
		only_grasps = [g[0] for g in grasps]
		grasp_weights = []
		for grasp in only_grasps:
			wt = self.pu.grasp_score_grasp(grasp)
			grasp_weights.append(wt)
		grasp_weights = grasp_weights/np.sum(grasp_weights)
		msg = [(pt, wt) for pt,wt in zip(only_grasps, grasp_weights)]
		return msg 


	def stable_func(self, stables, blah=None, bloh=None):
		only_stables = [g[0] for g in stables]
		stable_weights = []
		for stable in only_stables:
			wt = self.pu.stable_score_stable(stable)
			stable_weights.append(wt)
		stable_weights = stable_weights/np.sum(stable_weights)
		msg = [(pt, wt) for pt, wt in zip(only_stables, stable_weights)]
		return msg


	def se2_func(self, se2_traj, se2_pose, target):
		if target == "SE2-Trajectory":
			pmax = self.get_best(se2_pose)
			only_traj = [t[0] for t in se2_traj]
			traj_weights = []
			for traj in only_traj:
				wt = self.pu.se2_traj_score(traj, pmax)
				traj_weights.append(wt)
			traj_weights = traj_weights/np.sum(traj_weights)
			msg = [(pt,wt) for pt,wt in zip(only_traj, traj_weights)]
			return msg 

		elif target == "SE2-Pose":
			tmax = self.get_best(se2_traj)
			only_pose = [p[0] for p in se2_pose]
			pose_weights = []
			for pose in only_pose:
				wt = self.pu.se2_pose_score(tmax, pose)
				pose_weights.append(wt)
			pose_weights = pose_weights/np.sum(pose_weights)
			msg = [(pt,wt) for pt, wt in zip(only_pose, pose_weights)]
			return msg


	def get_best(self, particles):
		pwts = [p[1] for p in particles]
		max_part = particles[np.argmax(pwts)][0]
		return max_part


	def kin_func(self, grasps, confs, poses, target):
		if target == "Configuration":
			gmax = self.get_best(grasps)
			pmax = self.get_best(poses)
			only_confs = [q[0] for q in confs]
			conf_weights = []
			for conf in only_confs:
				wt = self.pu.kin_score_conf(conf, pmax, gmax)
				conf_weights.append(wt)
			conf_weights = conf_weights/np.sum(conf_weights)
			msg = [(pt,wt) for pt, wt in zip(only_confs, conf_weights)]
			return msg

		elif target == "Pose":
			gmax = self.get_best(grasps)
			cmax = self.get_best(confs)
			only_pose = [p[0] for p in poses]
			pose_weights=[]
			for pose in only_pose: 
				wt = self.pu.kin_score_poses(cmax, pose,gmax)
				pose_weights.append(wt)
			pose_weights = pose_weights/np.sum(pose_weights)
			msg = [(pt,wt) for pt, wt in zip(only_pose, pose_weights)]
			return msg

		elif target == "Grasp":
			pmax = self.get_best(poses)
			cmax = self.get_best(confs)
			only_grasps = [g[0] for g in grasps]
			grasp_weights = []
			for grasp in only_grasps:
				wt = self.pu.kin_score_grasps(cmax, pmax,grasp)
				grasp_weights.append(wt)
			grasp_weights = grasp_weights/np.sum(grasp_weights)
			msg = [(pt, wt) for pt, wt in zip(only_grasps, grasp_weights)]
			return msg

		else:
			return None  


class PMPNBP:
	def __init__(self, hcsp):
		self.hcsp = hcsp 


	def initialize_variables_with_prior(self, prior):
		print('initializing values')
		for constraint in self.hcsp.factor_graph:
			variables = self.hcsp.factor_graph[constraint][1] 
			for var in variables:
				var.initialize_with_prior(prior) 
		print('Done initializing')


	def pass_variables_to_constraint_msg(self):
		for constraint in self.hcsp.factor_graph:
			variables = self.hcsp.factor_graph[constraint][1] 
			const = self.hcsp.factor_graph[constraint][0] 
			for var in variables:
				msg = var.send_msg_to_constraint(const.name) 
				const.receive_msg_from_variable(var.type, msg) 

	def pass_constraint_to_variable_msg(self):
		for constraint in self.hcsp.factor_graph:
			variables = self.hcsp.factor_graph[constraint][1]
			const = self.hcsp.factor_graph[constraint][0] 
			for var in variables: 
				sampled_belief = var.get_sample_from_belief() 
				msg = const.send_msg_to_variable(sampled_belief, var.name, var.type)
				var.receive_msg_from_constraint(const.name, msg)

	def pass_messages_across_factor_graph(self, num_iterations=1):
		for ite in range(num_iterations):
			print('Iteration number: ',ite)
			self.pass_variables_to_constraint_msg()
			self.pass_constraint_to_variable_msg()

	def get_fleshed_actions(self):
		for act in self.hcsp.constrained_actions:
			const_ids = act.const_ids
			var = []
			for c in const_ids:
				varlist = self.hcsp.factor_graph[c][1]
				var += varlist
			act.variables = var
			act.get_map_for_variables()


		return self.hcsp.constrained_actions



		



'''
prior = {pear:[weigted particles of pear position],
		 stove-station: [weighted particles of stove se2 position],
		 wash-station: [weighted particles of wash se2 position],
		 tray: [weighted particles of tray position]}

'''

#hcsp.constrained actions has all actions. We can get their map values from them



if __name__ == '__main__':
	ps = Plan_skeleton([('get', 'pear'), ('wash','pear'), ('cook','pear')])
	# hcsp = ps.build_hcsp()	
	# ps.print_hcsp(hcsp) 
	# prior = self.pu.get_prior_belief(num_particles=50, targets=['pear', 'wash-station','stove-station','tray', 'wash-bowl','stove' ])
	# pmpnbp = PMPNBP(hcsp)
	# pmpnbp.initialize_variables_with_prior(prior)
	# pmpnbp.pass_messages_across_factor_graph()
	plan = ps.get_skeleton() 

	# plan = ps.plan_from('(handempty) (clean pear) (not (in-tray pear)) ',     '(cooked pear)')
	print('cook plan: ',plan)
	print('instruction index: ', ps.instruction_index)
	# print(ps.get_skeleton())

	'''
	skel = ps.assign_constraints_to_actions([('pick','pear') ,('put-on-tray','pear')])
	print(len(skel))
	for s in skel:
		print(s.name,len(s.constraints))
		'''