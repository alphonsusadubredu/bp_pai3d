import numpy as np 
import time
import pybullet_data
import pybullet as p
import pybullet_planning as pyplan
from pybullet_planning import Pose, Point
# from trac_ik_python.trac_ik import IK
import sys
import os
import grasps as gp
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
from utils.ikfast.digit_ik import solve_ik, solve_fk

def ikfunc(pose):
    return solve_ik(pose[0], pose[1], "right_arm")

class Buff_digit:
    def __init__(self, client):
        robot_id = p.loadURDF('/home/bill/garage/kfp_v2/digit/models/buff_digit/prost_digit_hover.urdf',[0,0,0.1],useFixedBase=True)
        self.client = client 
        self.id = robot_id
        self.state = "turn"
        self.max_angular_speed = 1.0
        self.max_linear_speed = 2.0
        self.turn_tol = 0.05
        self.trans_tol = 0.3
        self.final_trans_tol = 0.05
        self.left_wheel_id = 36
        self.right_wheel_id = 37
        self.wheel_width = 0.8
        self.arms_ee = {'left_arm':13, 'right_arm':26}
        self.arms_base = {'left_arm':'left_jlink0', 
                        'right_arm':'right_jlink0'}
        self.arm_joints = {'left_arm':[3,4,8,10,11,12], 
                           'right_arm':[16,17,21,23,24,25]}
        self.elbow_joints={'left_arm':(8,-1.35), 'right_arm':(21,1.35)}
        self.joint_index_ranges = {'left_arm':(0,6), 
                                   'right_arm':(6,12)} 
        self.grasped = {'left_arm':[], 'right_arm':[]}
        self.start_up_robot()
        time.sleep(5)
        
    def tuck_arm(self, armname='right_arm', right_side=False, left_side=True): 
        right_start_conf = (-1.3587102702612153, -0.9894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894)
        self.default_right_conf = right_start_conf
        left_start_conf = (0, 1.1894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894)
        if right_side:
            right_start_conf = [0, -1.1894200000000005, 1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if not left_side:
            left_start_conf = [-1.3587102702612153, 0.9894200000000005, -1.68495071580311, 0.20924737443538863, -0.0845840976133051, 0.20295805908247894]
        if armname == 'left_arm':
            self.drive_arm_joints(self.arm_joints[armname], left_start_conf)
        else:
            self.drive_arm_joints(self.arm_joints[armname], right_start_conf)



    def start_up_robot(self):
        self.lowerLimits,self.upperLimits,self.jointRanges,self.restPoses = self.get_joint_ranges()
        self.tuck_arm('left_arm')
        self.tuck_arm('right_arm') 
        self.get_camera_images()


    def angle_diff(self, angle1, angle2):
        diff = angle2 - angle1
        while diff < -np.pi: diff += 2.0*np.pi
        while diff > np.pi: diff -= 2.0*np.pi
        return diff

    def tf_arm_frame(self, pose, armname): 
        basename = self.arms_base[armname]
        baseid = pyplan.link_from_name(self.id, basename)
        base_to_world = pyplan.invert(pyplan.get_link_pose(self.id, baseid))
        base_to_pose = pyplan.multiply(base_to_world, pose)
        return base_to_pose 

    def tf_world_frame(self, base_to_pose, armname):
        basename = self.arms_base[armname]
        baseid = pyplan.link_from_name(self.id, basename)
        world_to_base = pyplan.get_link_pose(self.id, baseid)
        world_to_pose = pyplan.multiply(world_to_base, base_to_pose)
        return world_to_pose



    def follow_path(self, path):
        length = len(path)
        for i,pose in enumerate(path):
            self.move_to_pose(pose, last=(i==length-1))
        print('at goal pose')


    def plan_and_drive_to_pose_slide(self, goal_pose, limits=((-12.5, -12.5), (12.5, 12.5)) ,obstacles=[]): 
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        # self.follow_path(path)
        self.drive_along_path(path)

    def plan_to_pose(self, goal_pose, limits=((-12.5, -12.5), (12.5, 12.5)) ,obstacles=[]):
        path = pyplan.plan_base_motion(self.id, goal_pose, limits,obstacles=obstacles)
        return path 


    def plan_and_drive_to_pose(self, goal_pose,num_path = 50):
        current_pose = pyplan.get_base_values(self.id)
        target_heading = np.arctan2(goal_pose[1] - current_pose[1], goal_pose[0]-current_pose[0])
        init_ors = np.linspace(current_pose[2], target_heading,num_path)
        init_xs = [current_pose[0]]*num_path
        init_ys = [current_pose[1]]*num_path
        init_or_path = [(x,y,th) for x,y,th in zip(init_xs, init_ys, init_ors)]
        # print(init_or_path);print(' ')
        self.drive_along_path(init_or_path)

        current_pose = pyplan.get_base_values(self.id)
        xxs = np.linspace(current_pose[0],goal_pose[0],num_path*2)
        xarr = [current_pose[0], goal_pose[0]]
        yarr = [current_pose[1], goal_pose[1]]
        print(xarr,yarr)
        # yys = np.interp(xxs, xarr, yarr)
        yys = np.linspace(yarr[0], yarr[1], num_path*2)
        orrs = [current_pose[2]]*num_path*2
        trans_path = [(x,y,th) for x,y,th in zip(xxs,yys, orrs)]
        # print(trans_path);print(' ')
        self.drive_along_path(trans_path)

        current_pose = pyplan.get_base_values(self.id)
        goal_ors = np.linspace(current_pose[2], goal_pose[2], num_path)
        final_xs = [current_pose[0]]*num_path
        final_ys = [current_pose[1]]*num_path
        final_or_path = [(x,y,th) for x,y,th in zip(final_xs, final_ys, goal_ors)]
        self.drive_along_path(final_or_path)






    def drive_along_path(self, path, camera_follow=False):
        for pose in path:
            pyplan.set_base_values(self.id, pose) 
            if camera_follow:
                pose = pyplan.get_point(self.id)
                p.resetDebugVisualizerCamera(3, 90, -30, pose)
            time.sleep(0.05)

    
    def drive_base(self, linear_speed, angular_speed):
        left_vel = linear_speed - angular_speed*(self.wheel_width*0.5)
        right_vel = linear_speed + angular_speed*(self.wheel_width*0.5) 

        p.setJointMotorControl2(self.id, self.left_wheel_id, p.VELOCITY_CONTROL, targetVelocity=left_vel, force=1000)
        p.setJointMotorControl2(self.id, self.right_wheel_id, p.VELOCITY_CONTROL, targetVelocity=right_vel, force=1000)


    def move_to_pose(self, pose, last=False):
        orientation = pose[2]
        self.state = "turn"
        while not (self.state == "done"):
            current_pose = p.getBasePositionAndOrientation(self.id, self.client)[0]
            current_yaw = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id, self.client)[1])[2]
            target_heading = np.arctan2(pose[1] - current_pose[1], pose[0]-current_pose[0])
            error = self.angle_diff(target_heading, current_yaw)
            if orientation > 0.01:
                final_yaw_error = self.angle_diff(orientation, current_yaw)
            angular_speed = 0.0; linear_speed = 0.0
            # print(error)
            if self.state == "turn":
                if np.abs(error) > self.turn_tol:
                    linear_speed = 0.0
                    angular_speed = 0.0
                    if error > 0:
                        angular_speed = self.max_angular_speed
                    else:
                        angular_speed = -self.max_angular_speed

                else:
                    linear_speed = self.max_linear_speed
                    self.state = "drive"

            elif self.state == "drive":
                # self.tuck_arms()
                linear_speed = self.max_linear_speed
                dist_to_goal = np.linalg.norm(np.array(pose[:2])-np.array(current_pose[:2]))

                tol = self.trans_tol
                if last: tol = self.final_trans_tol
                
                if dist_to_goal < self.trans_tol:                   
                    linear_speed = 0.0
                    if orientation > 0.01:
                        self.state = "there"
                    else:
                        self.state = "done"
                        if not last:
                            linear_speed=self.max_linear_speed

            elif self.state == "there":
                if np.abs(final_yaw_error) > self.turn_tol:
                    linear_speed = 0.0
                    if final_yaw_error > 0.0:
                        angular_speed = self.max_angular_speed
                    else:
                        angular_speed = -self.max_angular_speed
                else:
                    self.state = "done"
            # print('Current position: %.2f %.2f %.2f'%(current_pose[0],current_pose[1],current_yaw))
            # print(self.state)
            self.drive_base(linear_speed, angular_speed)
        return True


    def get_joint_ranges(self, includeFixed=False):
        all_joints = pyplan.get_movable_joints(self.id)
        upper_limits = pyplan.get_max_limits(self.id, all_joints)
        lower_limits = pyplan.get_min_limits(self.id, all_joints)

        jointRanges, restPoses = [], []

        numJoints = p.getNumJoints(self.id)

        for i in range(numJoints):
            jointInfo = p.getJointInfo(self.id, i)

            if includeFixed or jointInfo[3] > -1:
                rp = p.getJointState(self.id, i)[0]
                jointRanges.append(2)
                restPoses.append(rp)

        return lower_limits, upper_limits, jointRanges, restPoses


    def drive_arm_joints(self, joints, jointPoses):
        assert(len(joints)==len(jointPoses))

        for i,j in enumerate(joints):
            p.setJointMotorControl2(bodyIndex=self.id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=jointPoses[i], maxVelocity=0.5)#, positionGain=0.001)
            time.sleep(0.01)
            p.stepSimulation()

    def move_arm_through_trajectory(self, trajectory, armname="right_arm"):
        joints = self.arm_joints[armname]
        for conf in trajectory:
            self.drive_arm_joints(joints, conf) 


    def plan_and_execute_arm_motion(self, position, orientation, armname='right_arm'): 
        pose = self.tf_arm_frame((position, orientation), armname) 
        gen = solve_ik(pose[0], pose[1],armname)
        a = next(gen)
        conf = next(gen) 

        if conf is not None:
            joints = self.arm_joints[armname] 
            self.drive_arm_joints(joints, conf)
        else:
            print('No IK solution found')

    def compute_ik_to_pose(self, position, orientation, armname):
        pose = self.tf_arm_frame((position, orientation), armname)
        try: 
            gen = solve_ik(pose[0], pose[1],armname)
            a = next(gen)
            conf = next(gen) 

            if conf is not None:
                return conf
            else: 
                return self.default_right_conf
        except: 
            return self.default_right_conf
 
    def get_generic_top_grasp(self, pose): 
        height = 0.1
        position, _ = pose
        position = list(position)
        position[2]+=(height/1.8)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        return position, orientation 

    def get_generic_side_grasp(self, pose):
        width = 0.1
        position, _ = pose 
        position = list(position)
        position[0] -= (width/2.0)
        orientation = p.getQuaternionFromEuler((1.57,0,0))
        return position, orientation 

    def get_top_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[2]+=(height/2.5)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        return position, orientation 

    def get_side_grasp(self, object_id):
        aabb = p.getAABB(object_id)
        width = aabb[1][0] - aabb[0][0]
        position, _ = pyplan.get_pose(object_id)
        position = list(position)
        position[0] -=(width/2.0)
        orientation = p.getQuaternionFromEuler((1.57,0,0))
        return position, orientation 

    def get_side_grasps(self, object_id):
        grasps = gp.get_side_grasps(object_id)
        transformed_grasps = []
        pose = pyplan.get_pose(object_id) 
        for g in grasps:
            gpose = pyplan.multiply(mugpose, pyplan.invert(g))
            transformed_grasps.append(gpose)
        return transformed_grasps


    def get_put_on_pose(self, topid, bottomid):
        bot_pose = pyplan.get_point(bottomid)
        aabb = p.getAABB(bottomid)
        botheight = aabb[1][2] - aabb[0][2]
        top_pose = list(bot_pose)
        top_pose[2] += botheight/1.8
        aabb = p.getAABB(topid)
        topheight = aabb[1][2] - aabb[0][2]
        top_pose[2] += topheight/1.8
        return top_pose


    def plan_arm_motion(self, pse, armname, obstacles=[]):
        #returns list of joint_confs and the path cost
        pose = self.tf_arm_frame(pse, armname) 
        current_conf = pyplan.get_joint_positions(self.id, self.arm_joints[armname])
        try:
            gen = solve_ik(pose[0], pose[1],armname) 
            a = next(gen)
            conf = next(gen)  

            if conf is not None:
                return [current_conf,conf]
            else: 
                return [current_conf, self.default_right_conf]
        except: 
            return [current_conf, self.default_right_conf]
             


    def forward_kinematics(self, conf, armname):
        pose_in_armbase_frame =  solve_fk(conf, armname)
        pose_in_world_frame = self.tf_world_frame(pose_in_armbase_frame, armname) 
        return pose_in_world_frame


    def get_traj_obst_cost(self, trajectory, armname, obstacles=[]):
        cost = 0
        for obs in obstacles:
            cloud = self.get_object_pointcloud(obs)
            for conf in trajectory:
                ee_position = list(self.forward_kinematics(conf, armname)[0])
                for point in cloud:
                    cost += np.linalg.norm(np.array(ee_position) - np.array(point))
        return cost

    def get_traj_goal_cost(self, trajectory, goal_pose, armname):
        conf = trajectory[-1]
        ee_position = self.forward_kinematics(conf, armname)[0]
        cost = np.linalg.norm(np.array(ee_position) - np.array(goal_pose[0]))
        return cost 

    def get_object_pointcloud(self, obid):
        cloud = []
        aabb = p.getAABB(obid)
        minx,miny,minz = aabb[0]
        maxx,maxy,maxz = aabb[1]
        num_x = max(int((maxx - minx)*50), 10)
        xs = np.linspace(minx, maxx, num=num_x)
        for px in xs:
            cloud.append([px, miny, minz])
            cloud.append([px, maxy, maxz]) 
            cloud.append([px, miny, maxz])
            cloud.append([px, maxy, minz])
        num_y = max(int((maxy - miny)*50), 10)
        ys = np.linspace(miny, maxy, num=num_y)
        for py in ys:
            cloud.append([minx, py, minz])
            cloud.append([maxx, py, maxz])
            cloud.append([minx, py, maxz])
            cloud.append([maxx, py, minz])
        num_z = max(int((maxz - minz)*50), 10)
        zs = np.linspace(minz, maxz, num=num_z)
        for pz in zs:
            cloud.append([minx, miny, pz])
            cloud.append([maxx, maxy, pz])
            cloud.append([minx, maxy, pz])
            cloud.append([maxx, miny, pz])
        
        return cloud

    def score_kin(self, conf, pose, grasp,armname='right_arm'): 
        ee_position = self.forward_kinematics(conf, armname)[0]
        cost = np.linalg.norm(np.array(ee_position) - np.array(grasp[0]))
        return cost

    def score_grasp(self, grasp):
        return 1

    def score_stable(self, placement):
        return 1

    def hold(self, object_id, armname='right_arm'):
        ee_id = self.arms_ee[armname] 
        self.grasped[armname].append(pyplan.add_fixed_constraint(object_id, self.id, ee_id))

    def release_hold(self, armname='right_arm'):
        for const in self.grasped[armname]:
            p.removeConstraint(const) 
        self.grasped[armname]=[]
        # p.removeConstraint(self.grasped[armname])

    def release_specific_hold(self, object_id, armname):
        ee_id = self.arms_ee[armname]
        pyplan.remove_fixed_constraint(object_id, self.id, ee_id)

    def raise_arm_after_pick(self, armname='right_arm'):
        pose = pyplan.get_link_pose(self.id, self.arms_ee[armname])
        position = list(pose[0])
        position[2]+=0.1 
        self.plan_and_execute_arm_motion(position, pose[1],armname)
        time.sleep(5)

    def pick_up(self, object_id, armname='right_arm'):
        grasp_position, grasp_orientation = self.get_top_grasp(object_id)
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)
        time.sleep(3) 
        ee_id = self.arms_ee[armname]
        ee_pose = pyplan.get_link_pose(self.id, ee_id)
        ob_pose = pyplan.get_point(object_id) 
        ob_point = list(ee_pose[0]); ob_point[2] = ob_pose[2] 
        pyplan.set_point(object_id, ob_point)
        self.hold(object_id, armname) 
        grasp_position[2] += 0.2
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)

    def pick_up_tray(self, object_id, armname):
        grasp_position, grasp_orientation = self.get_top_grasp(object_id)
        grasp_position = list(grasp_position); grasp_position[2]+=0.1;grasp_position[0]-=0.1; 
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)
        time.sleep(3) 
        self.hold(object_id, armname) 
        grasp_position[2] += 0.2
        self.plan_and_execute_arm_motion(grasp_position, grasp_orientation,armname)


    def place_at(self, position, object_id, armname='right_arm'):
        position = list(position)
        orientation = p.getQuaternionFromEuler((0,1.57,0))
        intermediate_position = position; intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        intermediate_position[2]-=0.2
        aabb = p.getAABB(object_id)
        height = aabb[1][2] - aabb[0][2]
        intermediate_position[2] += height/2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        self.release_hold(armname)
        time.sleep(2)
        intermediate_position[2]+=0.2
        self.plan_and_execute_arm_motion(intermediate_position,orientation,armname)
        time.sleep(2)
        # self.tuck_arm(armname)

    def get_camera_images(self): 
        eyeid = pyplan.link_from_name(self.id, 'torso_camera_link')
        fov, aspect, nearplane, farplane = 80, 1.0, 0.01, 100
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)
        com_p, com_o, _, _, _, _ = p.getLinkState(self.id, eyeid) 
        rot_matrix = p.getMatrixFromQuaternion(com_o)# 
        rot_matrix = np.array(rot_matrix).reshape(3, 3) 
        init_camera_vector = (1, 0, 0) # z-axis
        init_up_vector = (0, 1, 0) # y-axis
        # Rotated vectors
        camera_vector = rot_matrix.dot(init_camera_vector)
        up_vector = rot_matrix.dot(init_up_vector)
        view_matrix = p.computeViewMatrix(com_p+ 0.25 * camera_vector, com_p + 100 * camera_vector, up_vector)
        imgs = p.getCameraImage(640, 640, view_matrix, projection_matrix)
        return imgs


    def add_fixed_link_constraint(self, body, body_link, armname='right_arm',max_force=None):
        from pybullet_planning.interfaces.env_manager.pose_transformation import get_pose, unit_point, unit_quat, multiply, invert
        from pybullet_planning.interfaces.robots import get_com_pose, get_link_pose
 
        body_pose = get_link_pose(body, body_link) 
        ee = self.arms_ee[armname]
        end_effector_pose = get_com_pose(self.id, ee)
        grasp_pose = multiply(invert(end_effector_pose), body_pose)
        point, quat = grasp_pose 
        constraint = p.createConstraint(self.id, ee, body, body_link,  
                                        p.JOINT_FIXED, jointAxis=unit_point(),
                                        parentFramePosition=point,
                                        childFramePosition=unit_point(),
                                        parentFrameOrientation=quat,
                                        childFrameOrientation=unit_quat(),
                                        physicsClientId=self.client)
        if max_force is not None:
            p.changeConstraint(constraint, maxForce=max_force, physicsClientId=CLIENT)
        return constraint




    def open_drawer(self, cabinet, drawer_link, armname='right_arm'):
        drawer_pose = pyplan.get_link_pose(cabinet, drawer_link)
        drawer_position = list(drawer_pose[0])
        # robot_pose = list(pyplan.get_base_values(self.id))
        drawer_position[0] -= 0.3
        drawer_position[2] += 0.05
        grasp = self.get_generic_side_grasp((drawer_position,drawer_pose[1])) 
        self.plan_and_execute_arm_motion(grasp[0],grasp[1],armname)
        time.sleep(5) 
        drawer_position[0] -= 0.22
        grasp = self.get_generic_side_grasp((drawer_position,drawer_pose[1]))
        self.plan_and_execute_arm_motion(grasp[0],grasp[1],armname) 
        p.setJointMotorControl2(bodyIndex=cabinet, jointIndex=drawer_link, controlMode=p.POSITION_CONTROL, targetPosition=0.2, maxVelocity=0.5)
        time.sleep(5)

    def close_drawer(self, cabinet, drawer_link, armname='right_arm'):
        drawer_pose = pyplan.get_link_pose(cabinet, drawer_link)
        drawer_position = list(drawer_pose[0]) 
        drawer_position[0] -= 0.3
        drawer_position[2] += 0.05
        grasp = self.get_generic_side_grasp((drawer_position,drawer_pose[1])) 
        self.plan_and_execute_arm_motion(grasp[0],grasp[1],armname)
        time.sleep(5) 

        # drawer_position[0] += 0.22
        # grasp = self.get_generic_side_grasp((drawer_position,drawer_pose[1]))
        # self.plan_and_execute_arm_motion(grasp[0],grasp[1],armname) 
        p.setJointMotorControl2(bodyIndex=cabinet, jointIndex=drawer_link, controlMode=p.POSITION_CONTROL, targetPosition=0.0)#, maxVelocity=0.5)
        time.sleep(5)

    def press_dial(self, dial_position, armname='right_arm'):
        approach =  self.get_generic_top_grasp((dial_position,(0,0,0,1))) 
        self.plan_and_execute_arm_motion(approach[0], approach[1], armname)
        time.sleep(5)
        self.raise_arm_after_pick()
        time.sleep(2)


        


















if __name__ == '__main__':
    client = p.connect(p.GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER,1)
    # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME,0)

    p.setRealTimeSimulation(1)
    p.setGravity(0, 0, -9.81)
    p.setPhysicsEngineParameter(enableConeFriction=0)
    p.setAdditionalSearchPath('../models') 
    with pyplan.HideOutput(enable=True):
        floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)

        robot = Buff_digit(client)
        time.sleep(2)
        # stove_pose = [-4.0,0.5,-3.1415 ]
        # robot.diff_drive_to_pose(stove_pose)
        # robot.diff_drive_to_pose([0,0,0])
        # table = p.loadURDF('table/table.urdf',[0.8,0,0],p.getQuaternionFromEuler((0,0,-1.57)), useFixedBase=True)
        mug = p.loadURDF('mug/mug.urdf', [0.5,-0.3,0.9],useFixedBase=True)
   
    grasp = gp.get_side_grasps(mug)
    mugpose = pyplan.get_pose(mug)
    print('mug pose length: ',len(grasp));print('')
    for g in grasp:
        gpose = pyplan.multiply(mugpose, pyplan.invert(g))
        try:
            robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
        except:
            print('grasp failed')
        print(gpose)
        time.sleep(10)
        robot.tuck_arm()
        time.sleep(3)

    grasp = gp.get_top_grasps(mug) 
    print('mug pose length: ',len(grasp));print('')
    for g in grasp:
        gpose = pyplan.multiply(mugpose, pyplan.invert(g))
        try:
            robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
        except:
            print('grasp failed')
        print(gpose)
        time.sleep(10)
        robot.tuck_arm()
        time.sleep(3)

    # grasp_gen = gp.get_side_cylinder_grasps(mug) 

    # for i in range(20):
    #     g = next(grasp_gen)
    #     gpose = pyplan.multiply(mugpose, pyplan.invert(g))
    #     try:
    #         robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
    #     except:
    #         print('grasp failed')
    #     print(gpose)
    #     time.sleep(10)
    #     robot.tuck_arm()
    #     time.sleep(3)

    # print('cylinder top grasps of length : ',len(grasp))
    # for g in grasp:
    #     gpose = pyplan.multiply(mugpose, pyplan.invert(g))
    #     try:
    #         robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
    #     except:
    #         print('grasp failed')
    #     print(gpose)
    #     time.sleep(10)
    #     robot.tuck_arm()
    #     time.sleep(3)

    # grasp = gp.get_side_cylinder_grasps(mug)
    # mugpose = pyplan.get_pose(mug)
    # print('cylinder side grasps of length : ',len(grasp));print('')
    # for g in grasp:
    #     gpose = pyplan.multiply(mugpose, pyplan.invert(g))
    #     try:
    #         robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
    #     except:
    #         print('grasp failed')
    #     print(gpose)
    #     time.sleep(10)
    #     robot.tuck_arm()
    #     time.sleep(3)
    
    # grasp = gp.get_edge_cylinder_grasps(mug)
    # mugpose = pyplan.get_pose(mug)
    # print('mug pose: ',mugpose);print('')
    # for g in grasp:
    #     gpose = pyplan.multiply(mugpose, pyplan.invert(g))
    #     try:
    #         robot.plan_and_execute_arm_motion(gpose[0], gpose[1])
    #     except:
    #         print('grasp failed')
    #     print(gpose)
    #     time.sleep(10)
    #     robot.tuck_arm()
    #     time.sleep(3)



 
    time.sleep(2000)