
# -*- coding: utf-8 -*-
"""
Created on 3 May  2022

@author: mfocchi
"""

from __future__ import print_function

import copy
import os
import rospy as ros
import threading
from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty
from termcolor import colored

#gazebo messages
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.msg import ModelState
#gazebo services
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsPropertiesRequest
from gazebo_msgs.srv import GetPhysicsProperties

# ros utils
import roslaunch
import rospkg
from gazebo_msgs.srv import ApplyBodyWrench
import tf

#other utils
from base_controllers.utils.ros_publish import RosPub
from base_controllers.utils.pidManager import PidManager
from base_controllers.utils.math_tools import *
from numpy import nan
import matplotlib.pyplot as plt
from base_controllers.utils.common_functions import plotCoM, plotJoint
import pinocchio as pin
from base_controllers.utils.common_functions import getRobotModel

#dynamics
np.set_printoptions(threshold=np.inf, precision = 5, linewidth = 1000, suppress = True)
import  params as conf
robotName = "shelfino"


class BaseController(threading.Thread):
    """
        This Class can be used to simulate floating base robots that
        have an under-actuated base (e.g. like quadrupeds, mobile robots)

        ...

        Attributes
        ----------
        q, q_des : numpy array
           actual /desired joint positions
        qd, qd_des : numpy array
            actual /desired joint velocities
        tau, tau_ffwd :  numpy array
            actual /feed-forward torques
        basePoseW : numpy array
            base position (slice [0:3]) and orientation in Euler angles (slice [3:6])
        baseTwistW : numpy array
            base velocity linear (slice [0:3]) and angular (slice [3:6])
        b_R_w : numpy 2D array
            rotation matrix from world frame to base_link frame
        W_contacts : list of numpy arrays
            position of the feet expressed in the world frame
        grForcesW :  list of numpy arrays

        Methods
        -------
        loadModelAndPublishers(xacro_path=None)
           loads publishers for visual features (ros_pub), joint commands and declares subscriber to /ground_truth and /joint_states
        startSimulator(world_name = None)
           Starts gazebo simulator with ros_impedance_controller
        send_des_jstate(self, q_des, qd_des, tau_ffwd)
            publishes /command topic with set-points for joint positions, velocities and feed-forward torques
        startupProcedure():
            initialize PD gains
        initVars()
            initializes class variables
        logData()
            fill in the X_log variables for plotting purposes, it needs to be called at every loop
        _receive_jstate(msg)
            callback associated to the joint state subscriber, fills in q, qd, tau arrays
        _receive_pose(msg)
            callback associated to the ground truth subscriber, fills in base pose, orientation
            (quaternion, Euler angles), and base velocity (twist), and publishes the fixed transform between world and base_link

    """
    
    def __init__(self, robot_name="hyq", launch_file=None, external_conf = None):
        threading.Thread.__init__(self)
        if (external_conf is not None):
            conf.robot_params = external_conf.robot_params
        self.robot_name = robot_name
        self.base_offset = np.array([conf.robot_params[self.robot_name]['spawn_x'],
                                     conf.robot_params[self.robot_name]['spawn_y'],
                                     conf.robot_params[self.robot_name]['spawn_z']])

        self.joint_names = conf.robot_params[self.robot_name]['joint_names']
        self.u = Utils()
        self.math_utils = Math()
        # send data to param server
        self.verbose = conf.verbose
        self.custom_launch_file = False
        self.use_ground_truth_contacts = False
        self.apply_external_wrench = False
        self.time_external_wrench = 0.6
        self.broadcaster = tf.TransformBroadcaster()
        self.use_torque_control = False

        print("Initialized basecontroller---------------------------------------------------------------")

    def startSimulator(self, world_name = None, additional_args = None):
        # needed to be able to load a custom world file
        print(colored('Adding gazebo model path!', 'blue'))
        custom_models_path = rospkg.RosPack().get_path('ros_impedance_controller')+"/worlds/models/"
        if os.getenv("GAZEBO_MODEL_PATH") is not None:
            os.environ["GAZEBO_MODEL_PATH"] +=":"+custom_models_path
        else:
            os.environ["GAZEBO_MODEL_PATH"] = custom_models_path

        # clean up previous process
        os.system("killall rosmaster rviz gzserver gzclient")

        if self.custom_launch_file:
            launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_' + self.robot_name + '.launch'
        else:
            launch_file = rospkg.RosPack().get_path('ros_impedance_controller') + '/launch/ros_impedance_controller_floating.launch'

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [launch_file,
                    'robot_name:=' + self.robot_name,
                    'spawn_x:=' + str(conf.robot_params[self.robot_name]['spawn_x']),
                    'spawn_y:=' + str(conf.robot_params[self.robot_name]['spawn_y']),
                    'spawn_z:=' + str(conf.robot_params[self.robot_name]['spawn_z'])]
        if world_name is not None:
            print(colored("Setting custom model: "+str(world_name), "blue"))
            cli_args.append('world_name:=' + str(world_name))
        if additional_args is not None:
            cli_args.append(additional_args)
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
        parent.start()
        ros.sleep(1.0)
        print(colored('SIMULATION Started', 'blue'))

    def loadModelAndPublishers(self, xacro_path=None):

        # Loading a robot model of robot (Pinocchio)
        if xacro_path is None:
            xacro_path = rospkg.RosPack().get_path(
                self.robot_name + '_description') + '/robots/' + self.robot_name + '.urdf.xacro'
        else:
            print("loading custom xacro path: ", xacro_path)
        self.robot = getRobotModel(self.robot_name, generate_urdf=True, xacro_path=xacro_path)

        # instantiating objects
        self.ros_pub = RosPub(self.robot_name, only_visual=True)
        self.sub_jstate = ros.Subscriber("/"+self.robot_name+"/joint_states", JointState, callback=self._receive_jstate, queue_size=1,  tcp_nodelay=True)
        self.pub_des_jstate = ros.Publisher("/command", JointState, queue_size=1, tcp_nodelay=True)
        # freeze base  and pause simulation service
        self.reset_world = ros.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_physics_client = ros.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
        self.get_physics_client = ros.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        self.pause_physics_client = ros.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_physics_client = ros.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.u.putIntoGlobalParamServer("verbose", self.verbose)
        self.apply_body_wrench = ros.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

        self.sub_pose = ros.Subscriber("/" + self.robot_name + "/ground_truth", Odometry, callback=self._receive_pose,
                                       queue_size=1, tcp_nodelay=True)
        if self.use_ground_truth_contacts:
            self.sub_contact_lf = ros.Subscriber("/" + self.robot_name + "/lf_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_lf, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_rf = ros.Subscriber("/" + self.robot_name + "/rf_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_rf, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_lh = ros.Subscriber("/" + self.robot_name + "/lh_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_lh, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)
            self.sub_contact_rh = ros.Subscriber("/" + self.robot_name + "/rh_foot_bumper", ContactsState,
                                                 callback=self._receive_contact_rh, queue_size=1, buff_size=2 ** 24,
                                                 tcp_nodelay=True)

    def _receive_contact_lf(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] =  msg.states[0].wrenches[0].force.y
        grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(0, grf, self.grForcesLocal_gt)

    def _receive_contact_rf(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] =  msg.states[0].wrenches[0].force.y
        grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(1, grf, self.grForcesLocal_gt)

    def _receive_contact_lh(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] =  msg.states[0].wrenches[0].force.y
        grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(2, grf, self.grForcesLocal_gt)

    def _receive_contact_rh(self, msg):
        grf = np.zeros(3)
        grf[0] = msg.states[0].wrenches[0].force.x
        grf[1] =  msg.states[0].wrenches[0].force.y
        grf[2] =  msg.states[0].wrenches[0].force.z
        self.u.setLegJointState(3, grf, self.grForcesLocal_gt)

    def _receive_pose(self, msg):
        
        self.quaternion[0]=    msg.pose.pose.orientation.x
        self.quaternion[1]=    msg.pose.pose.orientation.y
        self.quaternion[2]=    msg.pose.pose.orientation.z
        self.quaternion[3]=    msg.pose.pose.orientation.w
        self.euler = euler_from_quaternion(self.quaternion)

        self.basePoseW[self.u.sp_crd["LX"]] = msg.pose.pose.position.x
        self.basePoseW[self.u.sp_crd["LY"]] = msg.pose.pose.position.y
        self.basePoseW[self.u.sp_crd["LZ"]] = msg.pose.pose.position.z
        self.basePoseW[self.u.sp_crd["AX"]] = self.euler[0]
        self.basePoseW[self.u.sp_crd["AY"]] = self.euler[1]
        self.basePoseW[self.u.sp_crd["AZ"]] = self.euler[2]

        self.baseTwistW[self.u.sp_crd["LX"]] = msg.twist.twist.linear.x
        self.baseTwistW[self.u.sp_crd["LY"]] = msg.twist.twist.linear.y
        self.baseTwistW[self.u.sp_crd["LZ"]] = msg.twist.twist.linear.z
        self.baseTwistW[self.u.sp_crd["AX"]] = msg.twist.twist.angular.x
        self.baseTwistW[self.u.sp_crd["AY"]] = msg.twist.twist.angular.y
        self.baseTwistW[self.u.sp_crd["AZ"]] = msg.twist.twist.angular.z

        # compute orientation matrix                                
        self.b_R_w = self.math_utils.rpyToRot(self.euler)
        self.broadcaster.sendTransform(self.u.linPart(self.basePoseW),
                                       self.quaternion,
                                       ros.Time.now(), '/base_link', '/world')

    def _receive_jstate(self, msg):
        for msg_idx in range(len(msg.name)):
            for joint_idx in range(len(self.joint_names)):
                if self.joint_names[joint_idx] == msg.name[msg_idx]:
                    self.q[joint_idx] = msg.position[msg_idx]
                    self.qd[joint_idx] = msg.velocity[msg_idx]
                    self.tau[joint_idx] = msg.effort[msg_idx]

    def send_des_jstate(self, q_des, qd_des, tau_ffwd):
         # No need to change the convention because in the HW interface we use our conventtion (see ros_impedance_contoller_xx.yaml)
         msg = JointState()
         msg.position = q_des
         msg.velocity = qd_des
         msg.effort = tau_ffwd                
         self.pub_des_jstate.publish(msg)     


    def deregister_node(self):
        print( "deregistering nodes"     )
        self.ros_pub.deregister_node()
        os.system(" rosnode kill /"+self.robot_name+"/ros_impedance_controller")    
        os.system(" rosnode kill /gazebo")   
      
 
    def get_contact(self):
        return self.W_contacts
    def get_pose(self):
        return self.basePoseW
    def get_jstate(self):
        return self.q
        
    def resetGravity(self, flag):
        # get actual configs
        physics_props = self.get_physics_client()         
       
        req_reset_gravity = SetPhysicsPropertiesRequest()
        #ode config
        req_reset_gravity.time_step = physics_props.time_step
        req_reset_gravity.max_update_rate = physics_props.max_update_rate           
        req_reset_gravity.ode_config =physics_props.ode_config
        req_reset_gravity.gravity =  physics_props.gravity
       
        
        if (flag):
            req_reset_gravity.gravity.z =  -0.2
        else:
            req_reset_gravity.gravity.z = -9.81                
        self.set_physics_client(req_reset_gravity)
        
    def freezeBase(self, flag):
        
        self.resetGravity(flag) 
        # create the message
        req_reset_world = SetModelStateRequest()
        #create model state
        model_state = ModelState()        
        model_state.model_name = self.robot_name
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0        
        model_state.pose.position.z = 0.8

        model_state.pose.orientation.w = 1.0
        model_state.pose.orientation.x = 0.0       
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0                
                                
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0        
        model_state.twist.linear.z = 0.0
             
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0        
        model_state.twist.angular.z = 0.0
             
        req_reset_world.model_state = model_state
        # send request and get response (in this case none)
        self.reset_world(req_reset_world) 

    def mapBaseToWorld(self, B_var):
        W_var = self.b_R_w.transpose().dot(B_var) + self.u.linPart(self.basePoseW)                            
        return W_var
                                                                                                                                
    def updateKinematics(self):
        # q is continuously updated
        # to compute in the base frame  you should put neutral base
        b_X_w = motionVectorTransform(np.zeros(3), self.b_R_w)
        gen_velocities  = np.hstack((b_X_w.dot(self.baseTwistW),self.u.mapToRos(self.qd)))
        neutral_fb_jointstate = np.hstack(( pin.neutral(self.robot.model)[0:7], self.u.mapToRos(self.q)))
        pin.forwardKinematics(self.robot.model, self.robot.data, neutral_fb_jointstate, gen_velocities)
        pin.computeJointJacobians(self.robot.model, self.robot.data)  
        pin.updateFramePlacements(self.robot.model, self.robot.data)
        ee_frames = conf.robot_params[self.robot_name]['ee_frames']
        for leg in range(4):
            self.B_contacts[leg] = self.robot.framePlacement(neutral_fb_jointstate,  self.robot.model.getFrameId(ee_frames[leg]), update_kinematics=True ).translation
            self.W_contacts[leg] = self.mapBaseToWorld(self.B_contacts[leg].transpose())
            if self.use_ground_truth_contacts:
                self.w_R_lowerleg[leg] = self.b_R_w.transpose().dot(self.robot.data.oMf[self.lowerleg_index[leg]].rotation)

        for leg in range(4):
            leg_joints =  range(6+self.u.mapIndexToRos(leg)*3, 6+self.u.mapIndexToRos(leg)*3+3) 
            self.J[leg] = self.robot.frameJacobian(neutral_fb_jointstate,  self.robot.model.getFrameId(ee_frames[leg]), pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3,leg_joints]  
            self.wJ[leg] = self.b_R_w.transpose().dot(self.J[leg])


       # Pinocchio Update the joint and frame placements
        gen_velocities  = np.hstack((b_X_w.dot(self.baseTwistW),self.u.mapToRos(self.qd)))
        configuration = np.hstack(( self.u.linPart(self.basePoseW), self.quaternion, self.u.mapToRos(self.q)))

        self.h = pin.nonLinearEffects(self.robot.model, self.robot.data, configuration, gen_velocities) 
        self.h_joints = self.h[6:]  
        
        #compute contact forces
        self.estimateContactForces()
        
        # compute com / robot inertias
        self.comB = self.robot.robotComB(self.u.mapToRos(self.q))
        self.comPoseW = copy.deepcopy(self.basePoseW)
        self.comPoseW[self.u.sp_crd["LX"]:self.u.sp_crd["LX"]+3] = self.robot.robotComW(configuration) # + np.array([0.05, 0.0,0.0])
        W_base_to_com = self.u.linPart(self.comPoseW)  - self.u.linPart(self.basePoseW) 
        self.comTwistW = np.dot( motionVectorTransform( W_base_to_com, np.eye(3)),self.baseTwistW)
        
        self.centroidalInertiaB = self.robot.centroidalInertiaB(configuration, gen_velocities)
        self.compositeRobotInertiaB = self.robot.compositeRobotInertiaB(configuration)

    def estimateContactForces(self):           
        # estimate ground reaxtion forces from tau 
        for leg in range(4):
            try:
                grf = np.linalg.inv(self.wJ[leg].T).dot(self.u.getLegJointState(leg,  self.u.mapFromRos(self.h_joints)-self.tau ))
            except np.linalg.linalg.LinAlgError as error:
                grf = np.zeros(3)
            self.u.setLegJointState(leg, grf, self.grForcesW)
            if self.contact_normal[leg].dot(grf) >= conf.robot_params[self.robot_name]['force_th']:
                self.contact_state[leg] = True
            else:
                self.contact_state[leg] = False

            if self.use_ground_truth_contacts:
                grfLocal_gt = self.u.getLegJointState(leg,  self.grForcesLocal_gt)
                grf_gt = self.w_R_lowerleg[leg] @ grfLocal_gt
                self.u.setLegJointState(leg, grf_gt, self.grForcesW_gt)

    def applyForce(self, Fx, Fy, Fz, Mx, My, Mz, duration):
        from geometry_msgs.msg import Wrench, Point
        wrench = Wrench()
        wrench.force.x = Fx
        wrench.force.y = Fy
        wrench.force.z = Fz
        wrench.torque.x = Mx
        wrench.torque.y = My
        wrench.torque.z = Mz
        reference_frame = "world"  # you can apply forces only in this frame because this service is buggy, it will ignore any other frame
        reference_point = Point(x=0, y=0, z=0)
        try:
            self.apply_body_wrench(body_name=self.robot_name+"::base_link", reference_frame=reference_frame,
                                   reference_point=reference_point, wrench=wrench, duration=ros.Duration(duration))
        except:
            pass
                                 
    def startupProcedure(self):

            self.pid = PidManager(self.joint_names) #I start after cause it needs joint names filled in by receive jstate callback
            # set joint pdi gains
            self.pid.setPDjoints(conf.robot_params[self.robot_name]['kp'], conf.robot_params[self.robot_name]['kd'], np.zeros(self.robot.na))

            if (self.robot_name == 'hyq'):                        
                # these torques are to compensate the leg gravity
                self.gravity_comp = np.array(
                    [24.2571, 1.92, 50.5, 24.2, 1.92, 50.5739, 21.3801, -2.08377, -44.9598, 21.3858, -2.08365, -44.9615])
                                                        
                print("reset posture...")
                self.freezeBase(1)
                start_t = ros.get_time()
                while ros.get_time() - start_t < 1.0:
                    self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                    ros.sleep(0.01)
                if self.verbose:
                    print("q err prima freeze base", (self.q - self.q_des))
  
          
                print("put on ground and start compensating gravity...")
                self.freezeBase(0)                                                
                ros.sleep(0.5)
                if self.verbose:
                    print("q err pre grav comp", (self.q - self.q_des))
                                                        
                start_t = ros.get_time()
                while ros.get_time()- start_t < 1.0:
                    self.send_des_jstate(self.q_des, self.qd_des, self.gravity_comp)
                    ros.sleep(0.01)
                if self.verbose:
                    print("q err post grav comp", (self.q - self.q_des))
                                                        
                print("starting com controller (no joint PD)...")                
                self.pid.setPDs(0.0, 0.0, 0.0)
                self.use_torque_control = True
            
            if (self.robot_name == 'solo' or self.robot_name == 'aliengo'):
                start_t = ros.get_time()
                while ros.get_time() - start_t < 0.5:
                    self.send_des_jstate(self.q_des, self.qd_des, self.tau_ffwd)
                    ros.sleep(0.01)
                #self.pid.setPDs(0.0, 0.0, 0.0)                    
            print(colored("finished startup -- starting controller", "red"))

    def initVars(self):
        self.comPoseW = np.zeros(6)
        self.baseTwistW = np.zeros(6)
        self.stance_legs = np.array([True, True, True, True])
        self.centroidalInertiaB = np.identity(3)
        self.compositeRobotInertiaB = np.identity(3)
        self.q = np.zeros(self.robot.na)
        self.qd = np.zeros(self.robot.na)
        self.tau = np.zeros(self.robot.na)
        self.q_des = np.zeros(self.robot.na)
        self.quaternion = np.array([0., 0., 0., 1.])
        self.q_des = conf.robot_params[self.robot_name]['q_0']
        self.qd_des = np.zeros(self.robot.na)
        self.tau_ffwd = np.zeros(self.robot.na)
        self.gravity_comp = np.zeros(self.robot.na)
        self.b_R_w = np.eye(3)
        self.grForcesW = np.zeros(self.robot.na)
        self.grForcesLocal_gt = np.zeros(self.robot.na)
        self.grForcesW_gt = np.zeros(self.robot.na)
        self.basePoseW = np.zeros(6)
        self.J = [np.zeros((6, self.robot.nv))] * 4
        self.wJ = [np.eye(3)] * 4
        self.W_contacts = [np.zeros((3))] * 4
        self.B_contacts = [np.zeros((3))] * 4
        self.contact_state = np.array([False, False, False, False])
        self.contact_normal = [np.array([0., 0., 1.])]*4
        self.w_R_lowerleg =  [np.eye(3)] * 4

        #log vars
        self.basePoseW_log = np.empty((6, conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.baseTwistW_log = np.empty((6,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        self.q_des_log = np.empty((self.robot.na, conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.q_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan   
        self.qd_des_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.qd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] )) *nan                                  
        self.tau_ffwd_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan    
        self.tau_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))*nan                                  
        self.grForcesW_log = np.empty((self.robot.na,conf.robot_params[self.robot_name]['buffer_size'] ))  *nan 
        self.time_log = np.empty((conf.robot_params[self.robot_name]['buffer_size']))*nan
        self.constr_viol_log = np.empty((4,conf.robot_params[self.robot_name]['buffer_size'] ))*nan
        
        self.time = 0.0
        self.log_counter = 0

        # order: lf rf lh rh
        if self.use_ground_truth_contacts:
            self.lowerleg_index = [0]*4
            self.lowerleg_frame_names = []
            for f in self.robot.model.frames:
                if 'lower' in f.name:
                    self.lowerleg_frame_names.append(f.name)
            self.lowerleg_frame_names = self.u.mapLegListToRos(self.lowerleg_frame_names)

            for legid in self.u.leg_map.keys():
                leg = self.u.leg_map[legid]
                self.lowerleg_index[leg] =  self.robot.model.getFrameId(self.lowerleg_frame_names[leg])

    def logData(self):
        if (self.log_counter<conf.robot_params[self.robot_name]['buffer_size'] ):
            self.basePoseW_log[:, self.log_counter] = self.basePoseW
            self.baseTwistW_log[:, self.log_counter] =  self.baseTwistW
            self.q_des_log[:, self.log_counter] =  self.q_des
            self.q_log[:,self.log_counter] =  self.q  
            self.qd_des_log[:,self.log_counter] =  self.qd_des
            self.qd_log[:,self.log_counter] = self.qd                       
            self.tau_ffwd_log[:,self.log_counter] = self.tau_ffwd                    
            self.tau_log[:,self.log_counter] = self.tau                     
            self.grForcesW_log[:,self.log_counter] =  self.grForcesW
            self.time_log[self.log_counter] = self.time
            self.log_counter+=1
	
def talker(p):
    p.start()
    if  (p.robot_name == 'solo_fw'):
        p.custom_launch_file = True
    p.startSimulator()
    p.loadModelAndPublishers()
    p.initVars()           
    p.startupProcedure()

    #loop frequency       
    rate = ros.Rate(1/conf.robot_params[p.robot_name]['dt']) 
    
#    ros.sleep(0.1)
#    p.resetGravity(True) 
#    
#    print ("Start flight phase")
    
#    p.time = 0.0
#    RPM2RAD =2*np.pi/60.0
#    omega = 5000*RPM2RAD

    #control loop
    while not ros.is_shutdown():
        #update the kinematics
        p.updateKinematics()    

        # controller
        if p.use_torque_control:
            p.tau_ffwd = conf.robot_params[p.robot_name]['kp'] * np.subtract(p.q_des,   p.q)  - conf.robot_params[p.robot_name]['kd']*p.qd + p.gravity_comp
            #p.tau_ffwd[12:] =0.01 * np.subtract(p.q_des[12:],   p.q[12:])  - 0.001*p.qd[12:]
        else:
            p.tau_ffwd = np.zeros(p.robot.na)
        
        
        #        p.q_des[14] += omega *conf.robot_params[p.robot_name_]['dt']		
#        p.q_des[15] += -omega *conf.robot_params[p.robot_name_]['dt']	    

        #max torque
#        p.tau_ffwd[14]= 0.21
#        p.tau_ffwd[15] = -0.21
        
        p.send_des_jstate(p.q_des, p.qd_des, p.tau_ffwd)
          
	   # log variables
        p.logData()    
        
        # plot actual (green) and desired (blue) contact forces 
        for leg in range(4):
            p.ros_pub.add_arrow(p.W_contacts[leg], p.contact_state[leg]*p.u.getLegJointState(leg, p.grForcesW/(6*p.robot.robot_mass)),"green")
            if (p.use_ground_truth_contacts):
                p.ros_pub.add_arrow(p.W_contacts[leg], p.u.getLegJointState(leg, p.grForcesW_gt / (6 * p.robot.robot_mass)), "red")
        p.ros_pub.publishVisual()      				
  
#        if (p.time>0.5): 
#            print ("pitch", p.basePoseW[p.u.sp_crd["AY"]])
#            break;

        #wait for synconization of the control loop
        rate.sleep()     
       
        p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]['dt']]), 3) # to avoid issues of dt 0.0009999

        if (p.apply_external_wrench and p.time > p.time_external_wrench):
            print("START APPLYING EXTERNAL WRENCH")
            p.applyForce(0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.05)
            p.apply_external_wrench = False


if __name__ == '__main__':
    p = BaseController(robotName)
    try:
        talker(p)
    except (ros.ROSInterruptException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
        if conf.plotting:
            plotJoint('position',0, p.time_log, p.q_log, p.q_des_log, p.qd_log, p.qd_des_log, None, None, p.tau_log, p.tau_ffwd_log, p.joint_names)
            plotCoM('position', 1, p.time_log, None, p.basePoseW_log, None, p.baseTwistW_log, None, None)



