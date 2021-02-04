import numpy as np
import pybullet as p 
from bppai import Action, Variable_node, Constraint_node, HCSP, Plan_skeleton, PMPNBP
from planning_utils import utils   
import os, sys, time
sys.path.append('../kfp_v2/src')
sys.path.append('../kfp_v2/digit/src') 
from world import Dining_World
from buff_digit import Buff_digit 
from action_model import Action_Model

np.random.seed(42)
instructions = [('get', 'pear'), ('wash','pear'), ('cook','pear')]
objects_of_interest = ['pear', 'wash-station','stove-station','tray', 'wash-bowl','stove','wash-tray-pose' , 'stove-tray-pose']

if __name__ == '__main__':
	client = p.connect(p.GUI)
	p.setRealTimeSimulation(1)
	robot = Buff_digit(client)
	world = Dining_World()

	pu = utils(robot, world)
	
	ps = Plan_skeleton(instructions, pu)
	ps.get_skeleton() 
	done = False
	while not done:

		hcsp = ps.build_hcsp()	 
		prior = pu.get_prior_belief(num_particles=1000, targets=objects_of_interest) 
		pmpnbp = PMPNBP(hcsp)
		pmpnbp.initialize_variables_with_prior(prior)
		pmpnbp.pass_messages_across_factor_graph(num_iterations=5)
		buff_plan = pmpnbp.get_fleshed_actions() 

		am = Action_Model(buff_plan, robot, world,pu)
		result = am.execute_plan()
		if not result:
			start_from = 0
			for inst in ps.instruction_index:
				if ps.instruction_index[inst][0] > am.index:
					start_from = ps.instruction_index[inst][1]
					break
			print('REPLANNING from action: ',am.index ) 
			# instructions = instructions[start_from:]
			ps.skeleton = ps.skeleton[am.index-1:]
			print('\nremainder: ',ps.skeleton)
			break

		else:
			done = True 
			print('Done')
	time.sleep(1000)