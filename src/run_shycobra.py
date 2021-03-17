import numpy as np
import pybullet as p 
from bhcsp import Action, Variable_node, Constraint_node, HCSP, Plan_skeleton, PMPNBP
from planning_utils import utils  
import pybullet_planning as pyplan 
import os, sys, time
sys.path.append('../kfp_v2/src')
sys.path.append('../kfp_v2/digit/src') 
from world import Apartment_World
from buff_digit import Buff_digit 
from action_model import Action_Model 

np.random.seed(42) 
objects_of_interest = ['pear', 'wash-station','stove-station','tray', 'wash-bowl','stove','plate','mug-grasp','mug-pose','mug-surface-pose','mug-station']
drawers = ['top-left', 'top-right', 'middle']
 


def collective_shycobra(robot, world, instructions):
	planning_time = 0.0
	num_replans = 0

	pu = utils(robot, world)
	chosen_drawer = pu.choose_drawer()
	START = time.time()
	ps = Plan_skeleton(instructions, pu=pu, drawer=chosen_drawer)
	ps.get_skeleton() 
	planning_time+= time.time() - START
	done = False
	while not done:
		

		START = time.time()
		hcsp = ps.build_hcsp()	 
		prior = pu.get_prior_belief(num_particles=100, targets=objects_of_interest,sigma=0.1) 
		pmpnbp = PMPNBP(hcsp)
		pmpnbp.initialize_variables_with_prior(prior)
		pmpnbp.pass_messages_across_factor_graph(num_iterations=10)
		buff_plan = pmpnbp.get_fleshed_actions() 

		am = Action_Model(buff_plan, robot, world,pu)
		planning_time+= time.time() - START
		result = am.execute_plan()
		if am.not_in_drawer:
			pu.update_drawer_belief(chosen_drawer)
			chosen_drawer = pu.choose_drawer()
			START = time.time()
			ps = Plan_skeleton(instructions, pu=pu, drawer=chosen_drawer)
			ps.get_skeleton()
			planning_time+= time.time() - START
			num_replans +=1

		else:

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
				num_replans+=1

				# START = time.time()
				# ps = Plan_skeleton(instructions, pu=pu, drawer=chosen_drawer)
				# ps.get_skeleton() 
				# planning_time+= time.time() - START
				

			else:
				done = True 
				print('Task completed')
				print('Planning Time: ',planning_time)
				print('Num replans: ',num_replans)
	return (planning_time,num_replans)



def shycobra(robot, world, instructions):
	planning_time = 0.0
	num_replans = 0

	inits=[None, ' (holding pear) ',' (clean pear) (holding pear) ',' (cooked pear) (observed pear) (handempty) ']

	for i in range(len(instructions)):
		inst = [instructions[i]]
		# print(inst)
		pu = utils(robot, world)
		chosen_drawer = pu.choose_drawer()
		START = time.time()
		ps = Plan_skeleton(inst, pu=pu, drawer=chosen_drawer, init_state=inits[i])
		ps.get_skeleton() 
		planning_time+= time.time() - START
		done = False
		while not done:
			START = time.time()
			hcsp = ps.build_hcsp()	 
			prior = pu.get_prior_belief(num_particles=10, targets=objects_of_interest,sigma=0.0001) 
			pmpnbp = PMPNBP(hcsp)
			pmpnbp.initialize_variables_with_prior(prior)
			pmpnbp.pass_messages_across_factor_graph(num_iterations=5)
			buff_plan = pmpnbp.get_fleshed_actions() 

			am = Action_Model(buff_plan, robot, world,pu)
			planning_time+= time.time() - START
			result = am.execute_plan()
			if am.not_in_drawer:
				pu.update_drawer_belief(chosen_drawer)
				chosen_drawer = pu.choose_drawer()
				START = time.time()
				ps = Plan_skeleton(inst, pu=pu, drawer=chosen_drawer)
				ps.get_skeleton()
				planning_time+= time.time() - START
				num_replans +=1

			else:

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
					num_replans+=1
					

				else:
					done = True 
					print('Task completed')
					print('Planning Time: ',planning_time)
					print('Num replans: ',num_replans)
	return (planning_time,num_replans)


 



def run_experiments(argv, robot, world ):
	tasks = ['get_pear','wash','cook', 'serve']
	drawers=['top-right','top-left','middle']
	algorithms=['joint','marginal']
	instructions =  [('get', 'pear'), ('wash','pear'), ('cook','pear'), ('serve','pear')]
	
	drawer = drawers[int(argv[1])-1]
	task = tasks[int(argv[2])-1]
	alg = algorithms[int(argv[3])-1]
	inst = instructions[:(int(argv[2]))]
	world.put_items_in_drawer(drawer)

	if alg == 'joint':
		pt,nr = bhcsp(robot, world, inst)
	elif alg == 'marginal':
		pt,nr = marginal_bhcsp(robot,world,inst)
	else:
		pt,nr = run_ssreplan(robot,world,inst)
	f = open('data3/'+alg+'_'+task+'_'+str(int(argv[1]))+'.txt','w')
	data = ['Planning time: '+str(pt)+'\n', 'Num replans: '+str(nr)]
	f.writelines(data)
	f.close() 
	pass


def run_shycobra(robot, world):
	tasks = ['get_pear','wash','cook', 'serve']
	drawers=['top-right','top-left','middle']
	algorithms=['joint','marginal']
	instructions =  [('get', 'pear'), ('wash','pear'), ('cook','pear'), ('serve','pear')]
	
	drawer = drawers[1]
	task = tasks[3]  
	world.put_items_in_drawer(drawer)

	_,_ = shycobra(robot, world, instructions)
	print('************Task complete***************')


if __name__ == '__main__':
	client = p.connect(p.GUI,options='--background_color_red=0.0 --background_color_green=0.0 --background_color_blue=0.0')
	p.configureDebugVisualizer(p.COV_ENABLE_GUI,0) 	
	
	with pyplan.HideOutput(enable=True):
		p.setGravity(0, 0, -9.81)
		world = Apartment_World()
		robot = Buff_digit(client)
	p.setRealTimeSimulation(1)  
	time.sleep(5)
	world.set_up()

	args = sys.argv
	# run_experiments(args, robot, world )
	run_shycobra(robot, world)
	time.sleep(20) 



