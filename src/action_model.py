import numpy as np 
import time
import pybullet_planning as pyplan
import pybullet as p


class Action_Model:
	def __init__(self, buff_plan, robot, world,pu):
		self.buff_plan = buff_plan
		self.robot =  robot
		self.world = world
		self.pu = pu
		self.not_in_drawer = False
		self.index = 0
		self.pear_in = False

	def execute_plan(self):
		for action in self.buff_plan:
			result = self.execute_action(action)
			self.index+=1

			if not result:
				return result
		return True

	def execute_action(self, action):
		success = True 
		print(action.name, action.obj)
		try:
			if action.name == 'open-drawer':
				self.robot.plan_and_drive_to_pose(self.world.cabinet_open_base_pose, self.world.base_limits) 
				self.robot.open_drawer(self.world.cabinet, self.world.get_drawer_id(action.obj[0]) ) 

				self.robot.raise_arm_after_pick() 
				current_pose = pyplan.get_base_values(self.robot.id)
				current_pose = list(current_pose)
				current_pose[0]+=0.2
				self.robot.plan_and_drive_to_pose(current_pose)
				time.sleep(5)
				return True


			elif action.name == 'inspect':
				if action.obj[1] != self.world.items_location:
					self.not_in_drawer = True
					print('id is ',self.world.get_drawer_id(action.obj[1]))
					current_pose = pyplan.get_base_values(self.robot.id)
					current_pose = list(current_pose)
					current_pose[0]-=0.2
					self.robot.plan_and_drive_to_pose(current_pose)
					self.robot.close_drawer(self.world.cabinet, self.world.get_drawer_id(action.obj[1]))
					# time.sleep(3)
					return False
				else:
					return True

			elif action.name == 'pick': 
				# self.robot.plan_and_drive_to_pose(self.world.cabinet_open_base_pose, self.world.base_limits)
				# current_pose = pyplan.get_base_values(self.robot.id)
				# current_pose = list(current_pose)
				# current_pose[0]+=0.2
				# self.robot.plan_and_drive_to_pose(current_pose)
				self.robot.raise_arm_after_pick()
				trajectory = action.maps['Trajectory'] 
				# pose = action.maps['Pose']
				# grasp = self.pu.compute_generic_grasp(pose)
				self.robot.move_arm_through_trajectory(trajectory) 
				# self.robot.plan_and_execute_arm_motion(grasp[0],grasp[1])
				time.sleep(5) 
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = pyplan.get_point(self.world.get_id(action.obj[0]))
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to pick target: ',dist)
				success = success and (dist < 0.1)
				if not success: 
					self.robot.raise_arm_after_pick()
					time.sleep(1)
					return False  
				
				obid = self.world.get_id(action.obj[0]) 
				self.robot.hold(obid)
				self.robot.raise_arm_after_pick()
				self.robot.raise_arm_after_pick() 

				# current_pose = pyplan.get_base_values(self.robot.id)
				# current_pose = list(current_pose)
				# current_pose[0]-=0.2
				# self.robot.plan_and_drive_to_pose(current_pose)
				# time.sleep(5)
				    

				


			elif action.name == 'go-to-wash-station':
				# # path = action.maps['SE2-Trajectory']
				# # self.robot.drive_along_path(path)
				# basepose = action.maps['SE2-Pose']
				# self.robot.plan_and_drive_to_pose(basepose)
				# # self.robot.plan_and_drive_to_pose(self.world.wash_station)
				
				# time.sleep(2)

				# basepose = pyplan.get_base_values(self.robot.id)
				# dist = np.linalg.norm((np.array(basepose[:2]) - np.array(self.world.wash_station[:2])))
				# print('dist to wash_station: ',dist)
				# grasp = self.pu.compute_generic_grasp(self.world.wash_tray_pose)
				# self.robot.plan_and_execute_arm_motion(grasp[0], grasp[1])
				# self.robot.release_hold()
				# self.robot.raise_arm_after_pick()
				# success = success and (dist < 0.2)
				# time.sleep(2)   
				# self.robot.raise_arm_after_pick()
				# current_pose = pyplan.get_base_values(self.robot.id)
				# current_pose = list(current_pose)
				# current_pose[0]-=0.4
				# current_pose[1]-=0.7
				# self.robot.plan_and_drive_to_pose(current_pose)
				# # self.robot.close_drawer(self.world.cabinet, self.world.get_drawer_id('top-right'),armname='left_arm' )
				# current_pose = pyplan.get_base_values(self.robot.id)
				# current_pose = list(current_pose)
				# current_pose[0]-=0.4        
				# self.robot.plan_and_drive_to_pose(current_pose)
				# self.robot.tuck_arm('left_arm',left_side=True)
				# time.sleep(3)

				self.robot.plan_and_drive_to_pose(self.world.sink_base_pose)
				time.sleep(5)


			elif action.name == 'wash':  
				
				self.robot.plan_and_drive_to_pose(self.world.sink_base_pose)
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				pose = action.maps['Pose'] 
				self.robot.plan_and_execute_arm_motion(pose[0],pose[1])
				time.sleep(3)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = self.world.sink_bottom_pose
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to wash target: ',dist)
				success = success and (dist < 0.1)
				if not success:
					time.sleep(1) 
					return False   
				time.sleep(5)


			elif action.name == 'go-to-stove':
				# basepose = action.maps['SE2-Pose']
				# self.robot.plan_and_drive_to_pose(basepose)
				# time.sleep(2)

				# basepose = pyplan.get_base_values(self.robot.id)
				# dist = np.linalg.norm((np.array(basepose[:2]) - np.array(self.world.stove_station[:2])))
				# print('dist to stove_station: ',dist)
				# grasp = self.pu.compute_generic_grasp(self.world.stove_tray_pose)
				# self.robot.plan_and_execute_arm_motion(grasp[0], grasp[1])
				# self.robot.release_hold()
				# self.robot.raise_arm_after_pick()
				# success = success and (dist < 0.2)
				# time.sleep(2)
				self.robot.plan_and_drive_to_pose(self.world.stove_base_pose)
				time.sleep(5)


			elif action.name == 'cook':
				self.robot.plan_and_drive_to_pose(self.world.stove_base_pose) 
				time.sleep(2)
				current_pose = pyplan.get_base_values(self.robot.id)
				current_pose = list(current_pose)
				current_pose[1]-=0.1        
				self.robot.plan_and_drive_to_pose(current_pose)
				time.sleep(2)
				self.robot.press_dial(self.world.stove_dial)
				time.sleep(2)
				current_pose[1]+=0.2        
				self.robot.plan_and_drive_to_pose(current_pose)


			elif action.name == 'distribute':
				self.robot.plan_and_drive_to_pose(self.world.tray_base_pose)
				time.sleep(2)
				self.robot.place_at(self.world.tray_surface_pose, self.world.get_id('pear'))
				time.sleep(4)
				pyplan.set_point(self.world.get_id('pear'), self.world.tray_surface_pose)
				self.robot.pick_up_tray(self.world.tray, armname='right_arm')
				self.robot.hold(self.world.get_id('pear'))
				current_pose = pyplan.get_base_values(self.robot.id)
				current_pose = list(current_pose)
				current_pose[0]-=1        
				self.robot.plan_and_drive_to_pose(current_pose)
				self.robot.plan_and_drive_to_pose(self.world.diningtable_base_pose)
				time.sleep(3)
				self.robot.place_at(self.world.diningtable_surface_pose, self.world.tray) 
				time.sleep(3)
				self.robot.pick_up(self.world.get_id('pear'))
				time.sleep(3)
				# self.robot.plan_and_drive_to_pose(self.world.plate_base_pose)


			elif action.name == 'serve':
				self.robot.plan_and_drive_to_pose(self.world.plate_base_pose)
				trajectory = action.maps['Trajectory']
				self.robot.move_arm_through_trajectory(trajectory)
				# pose = action.maps['Pose']
				# self.robot.plan_and_execute_arm_motion(pose[0],pose[1])
				time.sleep(3)
				time.sleep(3)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = self.world.plate_surface_pose
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to plate target: ',dist)
				success = success and (dist < 0.1)
				if not success: return False

				self.robot.release_hold()
				time.sleep(3)
				self.robot.plan_and_drive_to_pose(self.world.diningtable_base_pose)
				self.robot.tuck_arm('right_arm',right_side=True)


			elif action.name == 'go-to-mug-station':
				if self.pear_in:
					self.robot.tuck_arm(flat=True)
					self.robot.plan_and_drive_to_pose(self.world.mug_base_pose)
					time.sleep(5)

			elif action.name == 'put-food-in-saucepan':
				self.pear_in = True
				self.robot.raise_arm_after_pick()
				self.robot.raise_arm_after_pick()
				self.robot.plan_and_drive_to_pose(self.world.stove_base_pose)
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				pose = action.maps['Pose']
				self.robot.plan_and_execute_arm_motion(pose[0],pose[1])
				time.sleep(3)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = self.world.stove_surface_pose
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to stove target: ',dist)
				success = success and (dist < 0.1)
				if not success: 
					time.sleep(1)
					return False

				self.robot.release_hold()
				time.sleep(2)
				current_pose = pyplan.get_base_values(self.robot.id)
				current_pose = list(current_pose)
				current_pose[1]-=0.1        
				self.robot.plan_and_drive_to_pose(current_pose)
				time.sleep(2)

			elif action.name == 'pour-water-in-saucepan':
				self.robot.plan_and_drive_to_pose(self.world.stove_base_pose)
				self.robot.pour(self.world.saucepan)
				time.sleep(3)

			elif action.name == 'put-back-cup':
				self.robot.plan_and_drive_to_pose(self.world.mug_base_pose)
				time.sleep(3)
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				# time.sleep(3)
				pose = action.maps['Pose']
				self.robot.plan_and_execute_arm_motion(pose[0],pose[1])
				time.sleep(3)
				

				eepose = pyplan.get_point(self.world.mug)
				obpose = self.world.mug_surface_pose
				dist = np.linalg.norm((np.array(eepose)[:2] - np.array(obpose)[:2]))
				print('dist to place mug target: ',dist)
				success = success and (dist < 0.2)
				if not success: 
					time.sleep(1)
					return False

				self.robot.release_hold()
				time.sleep(2) 


			elif action.name == 'fill-with-water':
				self.robot.plan_and_drive_to_pose(self.world.sink_base_pose)
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				pose = action.maps['Pose'] 
				self.robot.plan_and_execute_arm_motion(pose[0],pose[1])
				time.sleep(3)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = self.world.sink_bottom_pose
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to water target: ',dist)
				success = success and (dist < 0.1)
				if not success:
					time.sleep(1) 
					return False   
				time.sleep(5)


			elif action.name == 'get-cup':
				# self.robot.raise_arm_after_pick()
				# self.robot.raise_arm_after_pick()
				self.robot.plan_and_drive_to_pose(self.world.mug_base_pose)
				time.sleep(3)
				grasp = action.maps['Grasp']
				# pose = action.maps['Pose']
				pose = pyplan.get_pose(self.world.mug)
				print('using grasp: ', grasp)
				gpose = pyplan.multiply(pose, pyplan.invert(grasp))
				self.robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
				time.sleep(5)

				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				obpose = self.world.opt_pour_global[0]
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose)[:2]))
				print('dist to grasp mug target: ',dist)
				success = success and (dist < 0.1)
				if not success:
					time.sleep(1) 
					return False   
				pp = list(gpose[0]); pp[2] = pyplan.get_point(self.world.mug)[2]; pp[0]+=0.07
				pyplan.set_point(self.world.mug, pp)
				p.removeConstraint(self.world.mug_constraint)

				self.robot.hold(self.world.mug)
				time.sleep(1)
				self.robot.raise_arm_after_pick()
				time.sleep(2)









 

			return success

		except Exception as e:
			print('Exception: ',e)
			return False
