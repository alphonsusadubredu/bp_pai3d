import numpy as np
from bppai import Action, Variable_node, Constraint_node, HCSP, Plan_skeleton, PMPNBP

if __name__ == '__main__':
	ps = Plan_skeleton([('get', 'pear'), ('wash','pear'), ('cook','pear')])
	hcsp = ps.build_hcsp()	
	ps.print_hcsp(hcsp) 
	prior = pu.get_prior_belief(num_particles=50, targets=['pear', 'wash-station','stove-station','tray', 'wash-bowl','stove' ])
	pmpnbp = PMPNBP(hcsp)
	pmpnbp.initialize_variables_with_prior(prior)
	pmpnbp.pass_messages_across_factor_graph()