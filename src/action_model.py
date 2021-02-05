import numpy as np 
import time
import pybullet_planning as pyplan


class Action_Model:
	def __init__(self, buff_plan, robot, world,pu):
		self.buff_plan = buff_plan
		self.robot =  robot
		self.world = world
		self.pu = pu
		self.index = 0

	def execute_plan(self):
		for action in self.buff_plan:
			result = self.execute_action(action)
			self.index+=1
			if not result:
				return result

	def execute_action(self, action):
		success = True 
		print(action.name, action.obj)
		try:
			if action.name == 'pick': 
				trajectory = action.maps['Trajectory'] 
				# pose = action.maps['Pose']
				# grasp = self.pu.compute_generic_grasp(pose)
				self.robot.move_arm_through_trajectory(trajectory)
				# self.robot.plan_and_execute_arm_motion(grasp[0],grasp[1])
				time.sleep(5)
				obid = self.world.get_id(action.obj) 
				self.robot.hold(obid)
				self.robot.raise_arm_after_pick()  
				obpose = pyplan.get_pose(obid)
				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(obpose[0])[:2]))
				print('dist to pick target: ',dist)
				success = success and (dist < 0.2)
				if not success: self.robot.release_hold()
				time.sleep(2) 

			elif action.name == 'put-on-tray': 
				basepose = self.pu.get_tray_base_pose()
				self.robot.plan_and_drive_to_pose(basepose)
				time.sleep(3)
				trajectory = action.maps['Trajectory']
				self.robot.move_arm_through_trajectory(trajectory)
				# pose = action.maps['Pose']
				# grasp = self.pu.compute_generic_grasp(pose)
				# # position = list(grasp[0]); position[2]+=0.1
				# self.robot.plan_and_execute_arm_motion(grasp[0],grasp[1]) 
				time.sleep(5)
				self.robot.release_hold()
				self.robot.raise_arm_after_pick()

				obid = self.world.get_id(action.obj)
				obpose = pyplan.get_pose(obid)
				traypose = pyplan.get_pose(self.world.object_indices['tray'])
				dist = np.linalg.norm((np.array(traypose[0])[:2] - np.array(obpose[0])[:2]))
				print('dist to put-on-tray target: ',dist)
				success = success and (dist < 0.2)
				time.sleep(2) 
 
			elif action.name == 'carry-tray': 

				trajectory = action.maps['Trajectory']  
				self.robot.move_arm_through_trajectory(trajectory)

				# grasp = action.maps['Grasp']
				# grasp = self.pu.compute_generic_grasp(pose)
				# position = list(grasp[0]); position[2]-=0.1
				# self.robot.plan_and_execute_arm_motion(grasp[0],grasp[1]) 

				time.sleep(5)
				obid = self.world.get_id(action.obj)
				trayid = self.world.object_indices['tray']
				self.robot.hold(obid)
				self.robot.hold(trayid)
				self.robot.raise_arm_after_pick()

				eepose = pyplan.get_link_pose(self.robot.id, self.robot.arms_ee['right_arm'])
				traypose = pyplan.get_pose(self.world.object_indices['tray'])
				dist = np.linalg.norm((np.array(eepose[0])[:2] - np.array(traypose[0])[:2]))
				print('dist to carry-tray target: ',dist)
				success = success and (dist < 0.2)
				if not success: self.robot.release_hold()
				time.sleep(2)  

			elif action.name == 'go-to-wash-station':
				# path = action.maps['SE2-Trajectory']
				# self.robot.drive_along_path(path)
				basepose = action.maps['SE2-Pose']
				self.robot.plan_and_drive_to_pose(basepose)
				# self.robot.plan_and_drive_to_pose(self.world.wash_station)
				
				time.sleep(2)

				basepose = pyplan.get_base_values(self.robot.id)
				dist = np.linalg.norm((np.array(basepose[:2]) - np.array(self.world.wash_station[:2])))
				print('dist to wash_station: ',dist)
				grasp = self.pu.compute_generic_grasp(self.world.wash_tray_pose)
				self.robot.plan_and_execute_arm_motion(grasp[0], grasp[1])
				self.robot.release_hold()
				self.robot.raise_arm_after_pick()
				success = success and (dist < 0.2)
				time.sleep(2)   

			elif action.name == 'wash':  
				self.robot.plan_and_drive_to_pose(self.world.wash_station_bowl)
				time.sleep(2)
				pose = action.maps['Pose']
				grasp = self.pu.compute_generic_grasp(pose)
				self.robot.plan_and_execute_arm_motion(grasp[0],grasp[1])
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				time.sleep(2)
				self.robot.release_hold()
				time.sleep(2)
				success = True

			elif action.name == 'cook':
				# trajectory = action.maps['Trajectory']
				# self.robot.move_arm_through_trajectory(trajectory)
				# time.sleep(2)
				# self.robot.release_hold()
				# time.sleep(2)
				# success = True

				obid = self.world.get_id(action.obj)
				self.robot.pick_up(obid)
				time.sleep(2)
				stove_pose = [-4.1,0.2,-3.1415]
				self.robot.plan_and_drive_to_pose(stove_pose, self.world.base_limits, obstacles=[self.world.kitchen])
				time.sleep(2)
				place_pose =  [-4.8, 0.5,0.9] 
				self.robot.place_at(place_pose, obid)
				time.sleep(2)
				success = True

			elif action.name == 'go-to-stove':
				basepose = action.maps['SE2-Pose']
				self.robot.plan_and_drive_to_pose(basepose)
				time.sleep(2)

				basepose = pyplan.get_base_values(self.robot.id)
				dist = np.linalg.norm((np.array(basepose[:2]) - np.array(self.world.stove_station[:2])))
				print('dist to stove_station: ',dist)
				grasp = self.pu.compute_generic_grasp(self.world.stove_tray_pose)
				self.robot.plan_and_execute_arm_motion(grasp[0], grasp[1])
				self.robot.release_hold()
				self.robot.raise_arm_after_pick()
				success = success and (dist < 0.2)
				time.sleep(2)







			return success

		except Exception as e:
			print('Exception: ',e)
			return False
