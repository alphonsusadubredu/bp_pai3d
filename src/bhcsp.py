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
		self.pfs = {}

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
			if self.weighted_particles[c] is not None:
				all_particles+=self.weighted_particles[c]
		if len(all_particles) == 0:
			# print('in ',self.name,self.type,self.action_name)
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
		# print('max weight: ',self.action_name, self.type,weights[maxind])
		return maxpart[0]


	def initialize_with_prior(self, prior): 
		if self.action_target != '': 
			self.prior_pose_particles = prior[self.action_target]
			self.prior_particles = prior[self.action_target]
			
			if self.action_name == 'pick': 
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
						# conf_noisy = self.pu.get_conf_noise(conf)
						# for cf in conf_noisy:
						# 	pts.append((cf,wt))
					self.prior_particles = pts 

			elif self.action_name == 'wash':
				if self.type == "Pose":
					self.prior_particles = prior['wash-bowl']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']:
						# grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(pose) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
						# conf_noisy = self.pu.get_conf_noise(conf)
						# for cf in conf_noisy:
						# 	pts.append((cf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back()



			elif self.action_name == 'cook':
				if self.type == "Pose":
					self.prior_particles = prior['stove']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
						# conf_noisy = self.pu.get_conf_noise(conf)
						# for cf in conf_noisy:
						# 	pts.append((cf,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()

			elif self.action_name == 'put-food-in-saucepan':
				if self.type == "Pose":
					self.prior_particles = prior['stove']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
						# conf_noisy = self.pu.get_conf_noise(conf)
						# for cf in conf_noisy:
						# 	pts.append((cf,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()


			elif self.action_name == 'serve':
				if self.type == "Pose":
					self.prior_particles = prior['plate']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('dining-station')
					for (pose,wt) in prior['plate']:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('dining-station')
					for (pose,wt) in prior['plate']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('dining-station')
					for (pose,wt) in prior['plate']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt))
						# conf_noisy = self.pu.get_conf_noise(conf)
						# for cf in conf_noisy:
						# 	pts.append((cf,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()


				

		else:
			if self.action_name == 'fill-with-water':
				if self.type == "Pose":
					self.prior_particles = prior['wash-bowl']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']: 
						traj = self.pu.plan_arm_trajectory_to(pose) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('wash-station')
					for (pose,wt) in prior['wash-bowl']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back()


			elif self.action_name == 'go-to-wash-station':
				self.prior_pose_particles = prior['wash-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['wash-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					pose = self.prior_pose_particles[0]
					traj = 0#self.pu.plan_se2_motion_to(pose[0])
					for (pose,wt) in self.prior_pose_particles:
						# traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts

			elif self.action_name == 'go-to-stove':
				self.prior_pose_particles = prior['stove-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['stove-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					pose = self.prior_pose_particles[0]
					traj = 0#self.pu.plan_se2_motion_to(pose[0])
					for (pose,wt) in self.prior_pose_particles:
						# traj = self.pu.plan_se2_motion_to(pose)
						pts.append((traj,wt))
					self.prior_particles = pts  

			elif self.action_name == 'go-to-mug-station':
				self.prior_pose_particles = prior['mug-station']
				self.prior_particles = prior['mug-station']
				if self.type == "SE2-Pose":
					self.prior_particles = prior['mug-station']
				elif self.type == "SE2-Trajectory":
					pts = []
					pose = self.prior_pose_particles[0]
					traj = 0
					for (pose,wt) in self.prior_pose_particles: 
						pts.append((traj,wt))
					self.prior_particles = pts

			elif self.action_name == 'pour-water-in-saucepan':
				if self.type == "Pose":
					self.prior_particles = prior['stove']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back()

			elif self.action_name == 'put-back-cup':
				if self.type == "Pose":
					self.prior_particles = prior['mug-surface-pose']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('mug-station')
					for (pose,wt) in prior['mug-surface-pose']:
						# grasp = self.pu.compute_generic_grasp(pose)
						traj = self.pu.plan_arm_trajectory_to((pose[0],(0,0,0,1))) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('mug-station')
					for (pose,wt) in prior['mug-surface-pose']:
						grasp = self.pu.compute_generic_grasp(pose)
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration":
					pts = []
					self.pu.teleport_to('mug-station')
					for (pose,wt) in prior['mug-surface-pose']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back()

			elif self.action_name == 'get-cup':
				if self.type == "Pose" :
					self.prior_particles = prior['mug-grasp']
				elif self.type == "Trajectory":
					pts = []
					self.pu.teleport_to('mug-station')
					for (grasp,wt) in prior['mug-pose']: 
						traj = self.pu.plan_arm_trajectory_to(grasp) 
						pts.append((traj,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Grasp":
					pts = []
					self.pu.teleport_to('mug-station')
					for (grasp,wt) in prior['mug-grasp']: 
						pts.append((grasp,wt))
					self.prior_particles = pts 
					self.pu.teleport_back()
				elif self.type == "Configuration"  :
					pts = []
					self.pu.teleport_to('mug-station')
					for (pose,wt) in prior['mug-grasp']: 
						conf = self.pu.compute_ik_to_pose(pose)
						pts.append((conf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back() 

				elif self.type == "Pose2" :
					self.prior_particles = prior['stove']

				elif self.type == "Configuration2"  :
					pts = []
					self.pu.teleport_to('stove-station')
					for (pose,wt) in prior['stove']:
						grasp = self.pu.compute_generic_grasp(pose)
						conf = self.pu.compute_ik_to_pose(grasp)
						pts.append((conf,wt)) 
					self.prior_particles = pts 
					self.pu.teleport_back()



 



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

		elif self.name == 'GraspC':
			if variabletype == 'Configuration':
				msg = self.constraint_function(self.variable_msgs['Grasp'], sampled_belief, self.variable_msgs['Pose'],self.variable_msgs['Pose2'],self.variable_msgs['Configuration2'],target=variabletype)
			elif variabletype == 'Pose':
				msg = self.constraint_function(self.variable_msgs['Grasp'], self.variable_msgs['Configuration'], sampled_belief, self.variable_msgs['Pose2'],self.variable_msgs['Configuration2'], target=variabletype)

			elif variabletype == 'Grasp':
				msg = self.constraint_function(sampled_belief, self.variable_msgs['Configuration'], self.variable_msgs['Pose'],self.variable_msgs['Pose2'],self.variable_msgs['Configuration2'],target=variabletype)

			elif variabletype == 'Pose2':
				msg = self.constraint_function(self.variable_msgs['Grasp'], self.variable_msgs['Configuration'], self.variable_msgs['Pose'], sampled_belief,self.variable_msgs['Configuration2'], target=variabletype)

			elif variabletype == 'Configuration2':
				msg = self.constraint_function(self.variable_msgs['Grasp'], self.variable_msgs['Configuration2'], self.variable_msgs['Pose'],self.variable_msgs['Pose2'],sampled_belief,target=variabletype)
 






class HCSP: 
	constrained_actions = None
	factor_graph = {}
	skeleton = None


class Plan_skeleton:
	def __init__(self, instructions,pu=None,drawer=None,init_state=None):
		self.actions=[]
		self.instructions=instructions
		self.pu = pu
		self.constraints = {'CFree':[('T','Trajectory'),('p', 'Pose')],
							'Kin':[('p', 'Pose'),('g','Grasp'),('q','Configuration')],
							'Stable':[('p', 'Pose')],
							'Grasp':[('g', 'Grasp')],
							'GraspC':[('g', 'Grasp'), ('p', 'Pose'), ('q','Configuration'), ('p', 'Pose2'), ('q','Configuration2')],
							'CFree-move':[('M','SE2-Trajectory'),('MP','SE2-Pose')]}
		self.cfuncs = {'CFree': self.collision_free_func,
					   'Kin': self.kin_func,
					   'Stable':self.stable_func,
					   'Grasp':self.grasph_func,
					   'GraspC':self.grasp_func,
					   'CFree-move':self.se2_func}
		self.domain_path = 'pddl/kitchen_domain.pddl' 
		self.ingredients = ['pear','meat_can']#,'strawberry', 'meatcan']
		self.drawers = ['top-left', 'top-right', 'middle']
		self.constraints_of_actions = {
			 'pick':['CFree', 'Kin', 'Grasp'],
			 'put-on-tray':['CFree','Kin', 'Stable'],
			 'carry-tray':['CFree', 'Kin', 'Grasp'],
			 'wash':['CFree','Kin'],
			 'cook':['CFree','Kin', 'Stable'],
			 'go-to-wash-station':['CFree-move'],
			 'go-to-stove':['CFree-move'],
			 'serve':['CFree', 'Kin', 'Stable'],
			 'open-drawer':None,
			 'inspect':None,
			 'distribute':None,
			 'go-to-mug-station':['CFree-move'],
			 'get-cup':['GraspC','Kin','CFree'],
			 'fill-with-water':['CFree','Kin'],
			 'pour-water-in-saucepan':['CFree','Kin', 'Stable'],
			 'put-back-cup':['CFree','Kin', 'Stable'],
			 'put-food-in-saucepan':['CFree','Kin', 'Stable']

			}
		self.instruction_index = {}
		self.chosen_drawer = str(drawer)
		self.init_state = init_state


	def get_skeleton(self):
		#forward search to densify instructions
		prev_state = ' (robot-at-foodstuff-station) (handempty)  (location pear '+self.chosen_drawer+') '
		if self.init_state != None:
			prev_state = self.init_state
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
			prev_state = goal_state 
		self.skeleton = skeleton 
		return skeleton

		 
	def get_constraint_sketch(self, skeleton):
		pick = Action('pick','mug', ['CFree', 'Kin'], ['q','p','g','T'])
		place = Action('place','mug', ['CFree', 'Kin'], ['q','p','g','T'])
		pass

	def infer_goal_state(self, action):
		state = ''
		if action[0] == 'get': 
			state = ' (holding '+action[1]+')' 

		elif action[0] == 'wash': 
			state = ' (clean '+action[1]+') (holding '+action[1]+')' 

		elif action[0] == 'cook': 
			state = ' (cooked '+action[1]+') (observed '+action[1]+') (handempty)'

		elif action[0] == 'serve': 
			state = ' (served '+action[1]+')'


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
		definition += "- ingredient\n"

		for drawer in self.drawers:
			definition += drawer+" "
		definition += "- drawer)\n"

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
			act = Action(action[0],action[1:])
			constraints = self.constraints_of_actions[action[0]]
			cc=[]
			var=[]
			if constraints is not None:
				for c in constraints:
					func = self.cfuncs[c]
					constobj = Constraint_node(c, self.constraints[c], func)
					cc.append(constobj) 
			act.constraints=cc 
			action_skeleton.append(act)
		return action_skeleton

 


	def build_hcsp_from_constrained_action_skeleton(self, const_actions):
		hcsp = HCSP()
		hcsp.constrained_actions = const_actions
		index = 0
		for act in hcsp.constrained_actions:  
			for const in act.constraints: 
				var_objs = [Variable_node(n[0], n[1], act.name, act.obj[0],self.pu) for n in const.variables]
				hcsp.factor_graph[act.name+act.obj[0]+const.name+str(index)] = [const, var_objs]
				act.const_ids.append(act.name+act.obj[0]+const.name+str(index)) 
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


	def grasph_func(self, grasps, blah=None, bloh=None):
		only_grasps = [g[0] for g in grasps]
		grasp_weights = []
		for grasp in only_grasps:
			wt = self.pu.grasph_score_grasp(grasp)
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
				# wt = self.pu.se2_traj_score(traj, pmax)
				wt = 1/100
				traj_weights.append(wt)
			traj_weights = traj_weights/np.sum(traj_weights)
			msg = [(pt,wt) for pt,wt in zip(only_traj, traj_weights)]
			return msg 

		elif target == "SE2-Pose":
			tmax = self.get_best(se2_traj)
			only_pose = [p[0] for p in se2_pose]
			pose_weights = []
			for pose in only_pose:
				# wt = self.pu.se2_pose_score(tmax, pose)
				wt = 1/100
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


	def grasp_func(self, grasps, confs, poses, poses2, confs2, target):
		if target == "Configuration":
			gmax = self.get_best(grasps)
			pmax = self.get_best(poses)
			c2max = self.get_best(confs2)
			p2max = self.get_best(poses2)
			only_confs = [q[0] for q in confs]
			conf_weights = []
			for conf in only_confs:
				wt = self.pu.grasp_score_conf1(conf,gmax, pmax, p2max, c2max)
				conf_weights.append(wt)
			conf_weights = conf_weights/np.sum(conf_weights)
			msg = [(pt,wt) for pt, wt in zip(only_confs, conf_weights)]
			return msg

		elif target == "Pose":
			gmax = self.get_best(grasps)
			cmax = self.get_best(confs)
			c2max = self.get_best(confs2)
			p2max = self.get_best(poses2)
			only_pose = [p[0] for p in poses]
			pose_weights=[]
			for pose in only_pose: 
				wt = self.pu.grasp_score_pose1(cmax, gmax, pose,p2max, c2max)
				pose_weights.append(wt)
			pose_weights = pose_weights/np.sum(pose_weights)
			msg = [(pt,wt) for pt, wt in zip(only_pose, pose_weights)]
			return msg

		elif target == "Grasp":
			pmax = self.get_best(poses)
			cmax = self.get_best(confs)
			c2max = self.get_best(confs2)
			p2max = self.get_best(poses2)
			only_grasps = [g[0] for g in grasps]
			grasp_weights = []
			for grasp in only_grasps:
				wt = self.pu.grasp_score_grasp(cmax, grasp, pmax, p2max, c2max )
				grasp_weights.append(wt)
			grasp_weights = grasp_weights/np.sum(grasp_weights)
			msg = [(pt, wt) for pt, wt in zip(only_grasps, grasp_weights)]
			return msg

		elif target == "Pose2":
			gmax = self.get_best(grasps)
			cmax = self.get_best(confs)
			c2max = self.get_best(confs2)
			pmax = self.get_best(poses)
			only_pose = [p[0] for p in poses2]
			pose_weights=[]
			for pose in only_pose: 
				wt = self.pu.grasp_score_pose2(cmax, gmax, pmax, pose, c2max)
				pose_weights.append(wt)
			pose_weights = pose_weights/np.sum(pose_weights)
			msg = [(pt,wt) for pt, wt in zip(only_pose, pose_weights)]
			return msg

		elif target == "Configuration2":
			gmax = self.get_best(grasps)
			pmax = self.get_best(poses)
			cmax = self.get_best(confs2)
			p2max = self.get_best(poses2)
			only_confs = [q[0] for q in confs2]
			conf_weights = []
			for conf in only_confs:
				wt = self.pu.grasp_score_conf2(cmax,gmax, pmax, p2max, conf)
				conf_weights.append(wt)
			conf_weights = conf_weights/np.sum(conf_weights)
			msg = [(pt,wt) for pt, wt in zip(only_confs, conf_weights)]
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


	def get_fleshed_actions_no_map(self):
		for act in self.hcsp.constrained_actions:
			const_ids = act.const_ids
			var = []
			for c in const_ids:
				varlist = self.hcsp.factor_graph[c][1]
				var += varlist
			act.variables = var 
		return self.hcsp.constrained_actions


		



'''
prior = {pear:[weigted particles of pear position],
		 stove-station: [weighted particles of stove se2 position],
		 wash-station: [weighted particles of wash se2 position],
		 tray: [weighted particles of tray position]}

'''

#hcsp.constrained actions has all actions. We can get their map values from them



if __name__ == '__main__':
	ps = Plan_skeleton([('get', 'pear'), ('wash','pear'), ('cook','pear')],drawer='top-right')#, ('serve','pear')],drawer='top-right')
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