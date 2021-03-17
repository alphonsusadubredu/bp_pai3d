import os, sys 
import time
import pybullet as p
import pybullet_data
from pybullet_object_models import ycb_objects, graspa_layouts
import pybullet_planning as pyplan
import numpy as np
from pybullet_utils import gazebo_world_parser
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import grasps as gp

class Grocery_World:
    def __init__(self,seed=0):
        np.random.seed(seed)
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbPear', 'YcbTomatoSoupCan', 'YcbTennisBall', 'YcbScissors']#,'YcbPowerDrill',  'YcbMustardBottle']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        self.xrange = (0.65, 0.73)
        self.yrange = (-0.45, -0.25)
        self.base_limits = ((-22.5, -22.5), (22.5, 22.5))
        self.pick_base_pose = [0.2,-0.25,0]
        self.place_base_pose = [0.2,0.25,0]
        self.init_item_properties()
        
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
            # self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
            self.table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.dining = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=False)
            self.bag = p.loadURDF('bag/bag.urdf',[0.8,0.25,0.9], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.load_objects()
        p.setGravity(0, 0, -9.81)
            
    def init_item_properties(self):
        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()]) 
            
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,1], flags=flags)
        
        print(self.ob_idx)
        self.idx_obs = dict([(value, key) for key, value in self.ob_idx.items()]) 
        print(self.idx_obs)


    def get_random_table_space(self):
        x = np.random.uniform(self.xrange[0], self.xrange[1])
        y = np.random.uniform(self.yrange[0], self.yrange[1])
        z = 1.0
        return [x,y,z]


    def get_item_mass(self, itemname): 
        return self.masses[itemname]


    def get_id(self, itemname):
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden

    def get_name(self, obid):
        return self.real_name[self.idx_obs[obid]]

    def get_names_of_ids(self, ids):
        idx = []
        for idd in ids:
            idx.append(self.real_name[self.idx_obs[idd]])
        return idx


class Apt_Grocery_World:
    def __init__(self,seed=0):
        np.random.seed(seed)
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox', 'YcbMasterChefCan', 'YcbPear', 'YcbTomatoSoupCan', 'YcbTennisBall', 'YcbScissors']#,'YcbPowerDrill',  'YcbMustardBottle']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        
        self.base_limits = ((-60.5, -60.5), (60.5, 60.5))
        self.pick_base_pose = [0.2,-0.25,3.14]
        self.place_base_pose = [0.2,0.25,3.14] 
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir) 
        p.setAdditionalSearchPath(model_path+'/digit/models')
        floor = p.loadURDF('floor/black_floor.urdf',[0,0,0.05],useFixedBase=True)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        self.apartment_bodies = gazebo_world_parser.parseWorld( p, filepath =    "worlds/kitchen.world")
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1) 
        self.init_item_properties()
        
        with pyplan.HideOutput(enable=True): 
            self.table = p.loadURDF('table/table.urdf',self.table_pose, p.getQuaternionFromEuler([0,0,-1.57]), useFixedBase=True) 
            self.bag = p.loadURDF('bag/bag.urdf',self.bag_pose, p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.load_objects()
        p.setGravity(0, 0, -9.81)
            
    def init_item_properties(self):
        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()]) 
        self.table_pose = [5,-3, 0]
        self.xrange = (self.table_pose[0]+0.32, self.table_pose[0]+0.27)#(0.65, 0.73)
        self.yrange = (self.table_pose[1]+0.45, self.table_pose[1]+0.25)#(-0.45, -0.25)
        self.robot_init_pose = ((6.3, -3, 0.05),p.getQuaternionFromEuler((0,0, 3.1415)))
        self.bag_pose = (self.table_pose[0]+0.35, self.table_pose[1]-0.25,0.9)
            
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,1], flags=flags)
        
        print(self.ob_idx)
        self.idx_obs = dict([(value, key) for key, value in self.ob_idx.items()]) 
        print(self.idx_obs)


    def get_random_table_space(self):
        x = np.random.uniform(self.xrange[0], self.xrange[1])
        y = np.random.uniform(self.yrange[0], self.yrange[1])
        z = 1.0
        return [x,y,z]


    def get_item_mass(self, itemname): 
        return self.masses[itemname]


    def get_id(self, itemname):
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden

    def get_name(self, obid):
        return self.real_name[self.idx_obs[obid]]

    def get_names_of_ids(self, ids):
        idx = []
        for idd in ids:
            idx.append(self.real_name[self.idx_obs[idd]])
        return idx




class Dining_World:
    def __init__(self):
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir)
        p.setAdditionalSearchPath(model_path+'/digit/models')
        self.item_names=['YcbPottedMeatCan']#YcbPear, YcbPottedMeatCan', 'YcbMustardBottle',  'YcbMasterChefCan','YcbStrawberry']#,  'YcbTomatoSoupCan', 'YcbGelatinBox','YcbTennisBall', 'YcbPowerDrill', 'YcbScissors']
        kitchen_path = 'kitchen_description/urdf/kitchen_part_right_gen_convex.urdf'
        self.xrange = (0.65, 0.73)
        self.yrange = (-0.45, -0.25)
        self.base_limits = ((-22.5, -22.5), (22.5, 22.5))
        self.pick_base_pose = [0.2,-0.25,0]
        self.place_base_pose = [0.2,0.25,0]
        self.wash_station = [-1.1,-1.2,-1.57]
        self.wash_station_bowl = [-0.8, -1.2, -1.57]
        self.stove_station = [-4.3,1,-3.1415 ]
        self.wash_tray_pose = [[-1.1,-1.7,0.9],[0,0,0,1]]
        self.stove_tray_pose = [[-4.4,1,0.9 ],[0,0,0,1]]
        self.burner_position = [-4.8, 0.65,0.9] 
        with pyplan.HideOutput(enable=True):
            self.floor = p.loadURDF('floor/floor.urdf',useFixedBase=True)
            self.kitchen = p.loadURDF(kitchen_path,[-5,0,1.477],useFixedBase=True)
            self.main_table = p.loadURDF('table/table.urdf',[1.0,0,0], p.getQuaternionFromEuler([0,0,1.57]), useFixedBase=True)
            self.wash_table = p.loadURDF('table/table.urdf',[-1.0,-2.0,0], useFixedBase=True)
            self.wash_bowl= p.loadURDF('bag/bag.urdf',[-0.8,-1.7,0.9], useFixedBase=True)
            self.dtable1 = p.loadURDF('dinner_table/dinner_table.urdf',[-3.0,4.0,0.4], useFixedBase=True)
            self.dtable2 = p.loadURDF('dinner_table/dinner_table.urdf',[-3.0,8.0,0.4], useFixedBase=True)
            self.dtable3 = p.loadURDF('dinner_table/dinner_table.urdf',[0.0,6.0,0.4], useFixedBase=True)
            self.tray = p.loadURDF('tray/tray.urdf',[0.8,0.25,0.9],p.getQuaternionFromEuler([0,0,-1.57]) )
            self.load_objects()
        p.setGravity(0, 0, -9.81)
        self.object_indices = {"meat_can":self.ob_idx["YcbPottedMeatCan"],
        "tray":self.tray, "wash-station":self.wash_station, "stove-station":self.stove_station, "wash-bowl":self.wash_bowl, "stove":self.kitchen}
        self.init_item_properties()
            
             
    def load_objects(self, arr_id=1):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        self.ob_idx = {} 

        for item in self.item_names:
            x = np.random.uniform(self.xrange[0], self.xrange[1])
            y = np.random.uniform(self.yrange[0], self.yrange[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,1], flags=flags)
        
        # print(self.ob_idx)
    def init_item_properties(self):
        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()])

    def get_id(self, itemname):
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden


    def sample_pose_from_normal(self, mean_pose, sigma, num_particles):
        xs = np.random.normal(loc=mean_pose[0][0], scale=sigma ,size=num_particles)
        ys = np.random.normal(loc=mean_pose[0][1], scale=sigma ,size=num_particles)
        zs = np.random.normal(loc=mean_pose[0][2], scale=sigma ,size=num_particles)
        positions = [(x,y,mean_pose[0][2]) for x,y,z in zip(xs,ys,zs)]
        quats = [mean_pose[1] for _ in range(num_particles)]
        samples = [(pose, qts) for pose,qts in zip(positions, quats)]
        return samples


    def sample_se2_from_normal(self,mean, sigma, num_particles):
        xs = np.random.normal(loc=mean[0],scale=sigma, size=num_particles)
        ys = np.random.normal(loc=mean[1],scale=sigma, size=num_particles)
        positions = [(x,y,mean[2]) for x,y in zip(xs,ys)]
        return positions


    def get_prior_belief(self, num_particles, targets,sigma=0.001):
        prior = {}
        for item in targets:
            if item == "meat_can":
                mean_pose = pyplan.get_pose(self.object_indices['meat_can'])
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['meat_can'] = particles 

            elif item == 'wash-station':
                mean_pose = self.wash_station
                samples = self.sample_se2_from_normal(mean_pose, sigma, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-station'] = particles

            elif item == 'stove-station':
                mean_pose = self.stove_station 
                samples = self.sample_se2_from_normal(mean_pose, sigma, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove-station'] = particles

            elif item == 'tray':
                mean_pose = pyplan.get_pose(self.tray)
                position = list(mean_pose[0]); position[2]+=0.15 #; position[1]-=0.15; 
                samples = self.sample_pose_from_normal((position,mean_pose[1]),sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['tray'] = particles

            elif item == 'wash-bowl':
                mean_pose = pyplan.get_pose(self.wash_bowl)
                position = list(mean_pose[0]); position[2]+=0.1
                samples = self.sample_pose_from_normal((position, mean_pose[1]),sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-bowl'] = particles

            elif item == 'stove':
                mean_pose = [[-4.1,0.5,1 ],[0,0,0,1]]
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove'] = particles

            elif item == 'stove-tray-pose':
                mean_pose = [[-4.3,1,0.9 ],[0,0,0,1]]
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove-tray-pose'] = particles

            elif item == 'wash-tray-pose':
                mean_pose = [[-1.1,-1.7,0.9],[0,0,0,1]]
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-tray-pose'] = particles
        return prior 


class Apartment_World:
    def __init__(self, seed=0):
        model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir) 
        p.setAdditionalSearchPath(model_path+'/digit/models')
        floor = p.loadURDF('floor/black_floor.urdf',[0,0,0.05],useFixedBase=True)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0) 
        self.apartment_bodies = gazebo_world_parser.parseWorld( p, filepath =    "worlds/kitchen.world")
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1)
        with pyplan.HideOutput(enable=True):
            self.cabinet = p.loadURDF('cabinet/cabinet.urdf',[8.7,-1.8, 0.75], p.getQuaternionFromEuler((0,0,3.1415)), useFixedBase=True)
            self.apartment_bodies.append(self.cabinet)
        self.init_items()
        self.init_constants()
        self.saucepan = p.loadURDF('saucepan/saucepan.urdf',self.stove_surface_pose,useFixedBase=True)
        self.tray = p.loadURDF('tray/tray.urdf', self.tray_surface_pose)
        self.plate = p.loadURDF('saucepan/saucepan.urdf', self.plate_surface_pose, useFixedBase=True)
        self.stand = p.loadURDF('mug/stand.urdf', self.stand_surface_pose, useFixedBase=True)
        self.mug = p.loadURDF('mug/mug.urdf', self.mug_surface_pose)
        # self.saucepan_constraint =  self.add_surface_constraint(self.cabinet_id, self.saucepan, (self.stove_surface_pose,(0,0,0,1)),max_force=None)
        self.mug_constraint = self.add_surface_constraint(self.mug, self.stand) 

    def init_items(self):
        # x,y,z = (8.7,-1.8, 1.35) 
        xrang = (8.5,8.9)
        yrang = (-1.8,-2.0)
        items = ['YcbPear','YcbStrawberry','YcbPottedMeatCan', 'YcbGelatinBox','YcbTomatoSoupCan']
        flags = p.URDF_USE_INERTIA_FROM_FILE 

        self.ob_idx = {} 

        for item in items:
            x = np.random.uniform(xrang[0], xrang[1])
            y = np.random.uniform(yrang[0], yrang[1])
            self.ob_idx[item] = p.loadURDF(os.path.join(ycb_objects.getDataPath(), item, 'model.urdf'), [x,y,1.35], flags=flags)
        self.drawers = {
        'top-left': 'dof_rootd_Aa001_t',
        'top-right': 'dof_rootd_Aa002_t',
        'middle':'dof_rootd_Aa003_t'
        }

        self.real_name = {'YcbStrawberry':'strawberry','YcbPottedMeatCan':'meat_can', 'YcbGelatinBox':'gelatin_box', 'YcbMasterChefCan':'masterchef_can', 'YcbPear':'pear' , 'YcbMustardBottle':'mustard', 'YcbTomatoSoupCan':'soup_can', 'YcbTennisBall':'tennis_ball', 'YcbPowerDrill':'drill', 'YcbScissors':'scissors','saucepan':'saucepan'}
        self.masses = {'strawberry':'light','meat_can':'heavy', 'gelatin_box':'light', 'masterchef_can':'heavy', 'pear':'light' , 'mustard':'heavy', 'soup_can':'light', 'tennis_ball':'light', 'drill':'heavy', 'scissors':'light'}
        self.mesh_name = dict([(value, key) for key, value in self.real_name.items()])

    def get_id(self, itemname):
        print(itemname)
        mesh_name = self.mesh_name[itemname]
        iden = self.ob_idx[mesh_name]
        return iden

    def init_constants(self):
        self.joint_noise = 0.05
        self.cabinet_open_base_pose = (7.65, -1.6, 0)
        self.sink_base_pose = (8.3, -4.3, -1.57)
        self.mug_base_pose = (7.6, -4.3, -1.57)
        self.stove_base_pose = (8.21, -3.15, 0)
        self.sink_bottom_pose = (8.2, -5.05, 1.08)
        self.stove_surface_pose = (8.9, -3.15, 1.05)
        self.mug_surface_pose = (7.5,-4.95, 1.15)
        self.stand_surface_pose = (7.5,-4.95, 1)
        self.tray_surface_pose = (8.8, -2.5, 1.0)
        self.stove_dial = (8.85, -3.4, 0.95)
        self.dining_base_pose = (6.5,2.0,-1.57)#(5.0,1.0,0.0)
        self.cabinet_id = 40
        self.tray_base_pose2 = (8.21, -2.6, 0)
        self.tray_base_pose = (8.21, -2.6, 0)
        self.plate_surface_pose = (6.1,0.7,0.9)
        self.plate_base_pose = (5.5, 0.1, 1.57)
        self.diningtable_base_pose = (5.3,1,0)
        self.diningtable_surface_pose = (5.8,1.0,0.9)
        self.base_limits = ((-60, -60), (60, 60))
        self.opt_pour_global = ((7.450602054595947, -4.9565935134887695, 1.2782944440841675), (-3.829010708500391e-08, 1.2995128599868622e-06, 0.014017791487276554, 0.9999017715454102)) 
        self.opt_pour = ((0.048268500715494156, 0.007135413587093353, -0.1897524893283844), (0.0, 0.0, 0.0, 1.0))


    def get_noisy_pose(self, obname, sigma=0.2):
        obid = self.get_id(obname)
        pose = pyplan.get_point(obid)
        x = np.random.normal(loc=pose[0],scale=sigma)
        y = np.random.normal(loc=pose[1],scale=sigma)
        z = pose[2]
        return (x,y,z)

    def get_noisy_grasp(self, grasp, sigma=0.2):
        pose = grasp[0]
        x = np.random.normal(loc=pose[0],scale=sigma)
        y = np.random.normal(loc=pose[1],scale=sigma)
        z = pose[2]
        return (x,y,z)

    

    def add_surface_constraint(self, body, robot, robot_link=-1, max_force=None):
        from pybullet_planning.interfaces.env_manager.pose_transformation import get_pose, unit_point, unit_quat, multiply, invert
        from pybullet_planning.interfaces.robots import get_com_pose

        body_link = -1
        robot_link = -1
        body_pose = get_pose(body) 
        end_effector_pose = get_pose(robot)
        grasp_pose = multiply(invert(end_effector_pose), body_pose)
        point, quat = grasp_pose 
        constraint = p.createConstraint(robot, robot_link, body, body_link,  # Both seem to work
                                        p.JOINT_FIXED, jointAxis=unit_point(),
                                        parentFramePosition=point,
                                        childFramePosition=unit_point(),
                                        parentFrameOrientation=quat,
                                        childFrameOrientation=unit_quat() )
        if max_force is not None:
            p.changeConstraint(constraint, maxForce=max_force, physicsClientId=CLIENT)
        return constraint

    def put_items_in_drawer(self, drawer_name): 
        self.items_location = drawer_name
        link_id = self.get_drawer_id(drawer_name)
        pose = pyplan.get_link_pose(self.cabinet, link_id)
        point = list(pose[0])
        point[0]-=0.2
        for item in self.ob_idx.values():
            pyplan.set_point(item, point)


    def get_drawer_id(self, drawer_name):
        
        link_name = self.drawers[drawer_name]
        link_id = pyplan.link_from_name(self.cabinet, link_name)
        return link_id

    # def get_id(self, itemname):
    #     mesh_name = self.mesh_name[itemname]
    #     iden = self.ob_idx[mesh_name]
    #     return iden


    def sample_pose_from_normal(self, mean_pose, sigma, num_particles):
        xs = np.random.normal(loc=mean_pose[0][0], scale=sigma ,size=num_particles)
        ys = np.random.normal(loc=mean_pose[0][1], scale=sigma ,size=num_particles)
        zs = np.random.normal(loc=mean_pose[0][2], scale=sigma ,size=num_particles)
        positions = [(x,y,mean_pose[0][2]) for x,y,z in zip(xs,ys,zs)]
        quats = [mean_pose[1] for _ in range(num_particles)]
        samples = [(pose, qts) for pose,qts in zip(positions, quats)]
        return samples


    def sample_se2_from_normal(self,mean, sigma, num_particles):
        xs = np.random.normal(loc=mean[0],scale=sigma, size=num_particles)
        ys = np.random.normal(loc=mean[1],scale=sigma, size=num_particles)
        positions = [(x,y,mean[2]) for x,y in zip(xs,ys)]
        return positions


    def get_prior_belief(self, num_particles, targets,sigma=0.0001):
        prior = {}
        for item in targets:
            if item == "pear":
                mean_pose = pyplan.get_pose(self.ob_idx['YcbPear'])
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['pear'] = particles 

            elif item == 'wash-station':
                mean_pose = self.sink_base_pose
                samples = self.sample_se2_from_normal(mean_pose, 0.000001, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-station'] = particles

            elif item == 'stove-station':
                mean_pose = self.stove_base_pose
                samples = self.sample_se2_from_normal(mean_pose, 0.0000001, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove-station'] = particles


            elif item == 'wash-bowl': 
                position = self.sink_bottom_pose
                samples = self.sample_pose_from_normal((position, (0,0,0,1)),sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['wash-bowl'] = particles

            elif item == 'stove':
                position = list(self.stove_surface_pose)
                position[2]+=0.1
                mean_pose = (position,(0,0,0,1))
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['stove'] = particles

            elif item == 'plate': 
                position = self.plate_surface_pose
                samples = self.sample_pose_from_normal((position, (0,0,0,1)),sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['plate'] = particles

            elif item == 'mug-grasp': 
                samples = []
                grasps = gp.get_top_grasps(self.mug)
                samples+=grasps
                grasps = gp.get_side_grasps(self.mug)
                samples+=grasps  
                particles = [(pt,1/len(samples)) for pt in samples]
                prior['mug-grasp'] = particles

            elif item == "mug-pose":
                mean_pose = pyplan.get_pose(self.mug)
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['mug-pose'] = particles

            elif item == "mug-surface-pose":
                mean_pose =  (self.mug_surface_pose,(0,0,0,1)) 
                samples = self.sample_pose_from_normal(mean_pose,sigma,num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['mug-surface-pose'] = particles

            elif item == 'mug-station':
                mean_pose = self.mug_base_pose
                samples = self.sample_se2_from_normal(mean_pose, 0.000001, num_particles)
                particles = [(pt,0.02) for pt in samples]
                prior['mug-station'] = particles


        return prior



                



class Grocery_Bag:
    def __init__(self, bagid):
        self.id = bagid
        self.full_cpty = 4
        w = 0.25
        b = 0.2
        x, y, _ = pyplan.get_point(bagid)
        self.index = 0
        self.positions = [(x+b/5, y+w/5), (x+b/5, y-w/5), (x-b/5, y+w/5), (x-b/5,y-w/5)]
        self.radius = 0.00625
        self.z = 0.97
        self.occupancy = [0,0,0,0]
        self.items_added = {}
        self.bag_scene_graph = {}
        self.num_base_items = 0
        self.num_total_items = 0

    def add_item(self, itemid):
        if self.num_base_items >= 4:
            print('Bag full')
            return None
        cx, cy = self.positions[self.index]
        x = np.random.uniform(cx-self.radius, cx+self.radius)
        y = np.random.uniform(cy-self.radius, cy+self.radius)
        self.items_added[itemid] = self.index
        self.bag_scene_graph[itemid] = None
        self.occupancy[self.index] = 1 
        self.num_base_items +=1
        self.num_total_items +=1
        self.reposition_index()
        return (x,y,self.z)

    def add_on_item(self, topid, botid):
        self.items_added[topid] = 99
        self.bag_scene_graph[botid] = topid
        self.num_total_items +=1

    def remove_on_item(self, itemid):
        if itemid in self.items_added:
            self.items_added.pop(itemid)
            self.num_total_items -=1
            botid = self.get_item_under(itemid)
            self.bag_scene_graph[botid] = None

    def remove_item(self, itemid):
        if itemid in self.items_added:
            index = self.items_added[itemid]
            self.occupancy[index] = 0
            self.items_added.pop(itemid)
            self.num_base_items -=1
            self.num_total_items -=1
            self.reposition_index()

    def reposition_index(self):
        free = False
        for i in range(len(self.occupancy)):
            if self.occupancy[i] == 0:
                self.index = i 
                free = True
                break
        if not free:
            print('1 free space left')

    def get_ids_of_items(self):
        names = self.items_added.keys() 
        return names

    def get_itemid_on_top(self, obid):
        if obid in self.bag_scene_graph:
            topid = self.bag_scene_graph[obid]
            return topid
        #     if topid is None:
        #         return None
        #     else:
        #         return self.gw.get_name(topid)
        # else:
        #     return None

    def get_item_under(self, obid):
        for bot in self.bag_scene_graph:
            if self.bag_scene_graph[bot] == obid:
                return bot 
        return None










if __name__ == '__main__':
    # vhacd()
    # '''
    dining_world_test()
    # '''
