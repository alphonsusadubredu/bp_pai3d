import numpy as np 
import time
import pybullet_planning as pyplan


class Action_Model:
	def __init__(self, buff_plan, robot, world):
		self.buff_plan = buff_plan
		self.robot =  robot
		self.world = world


	def execute_plan(self):
		for action in self.buff_plan:
			if action.name == 'pick':
				trajectory = action.maps['Trajectory'] 
				self.robot.move_arm_through_trajectory(trajectory)
				time.sleep(5)
				obid = self.world.get_id(action.obj) 
				self.robot.hold(obid)
				self.robot.raise_arm_after_pick()  
				obpose = pyplan.get_pose(obid)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose[0])[:2]))
				print('dist to target: ',dist)
				break
