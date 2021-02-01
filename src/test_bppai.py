import numpy as np
import pybullet as p 
from bppai import Action, Variable_node, Constraint_node, HCSP, Plan_skeleton, PMPNBP
from planning_utils import utils   
import os, sys, time
sys.path.append('../kfp_v2/src')
sys.path.append('../kfp_v2/digit/src') 
from world import Dining_World
from buff_digit import Buff_digit 

instructions = [('get', 'pear'), ('wash','pear'), ('cook','pear')]
objects_of_interest = ['pear', 'wash-station','stove-station','tray', 'wash-bowl','stove' ]

if __name__ == '__main__':
	client = p.connect(p.GUI)
	p.setRealTimeSimulation(1)
	robot = Buff_digit(client)
	world = Dining_World()

	pu = utils(robot, world)
	ps = Plan_skeleton(instructions, pu)
	hcsp = ps.build_hcsp()	
	# ps.print_hcsp(hcsp) 
	prior = pu.get_prior_belief(num_particles=50, targets=objects_of_interest) 
	pmpnbp = PMPNBP(hcsp)
	pmpnbp.initialize_variables_with_prior(prior)
	pmpnbp.pass_messages_across_factor_graph(num_iterations=3)

	print('Done')
	time.sleep(1000)