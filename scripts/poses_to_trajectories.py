#!/usr/bin/env python

from __future__ import print_function
from __future__ import division
from builtins import zip
from builtins import map
from builtins import str
from builtins import range
from past.utils import old_div
from builtins import object
from itertools import repeat, starmap

import rospy
from nav_msgs.msg import Odometry
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, Transform, Pose, Point, Vector3, PoseArray
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from std_msgs.msg import Header
from nav_quadrotor.trajectories import ToGoal
from nav_quadrotor.state_conversions import MsgToState, StateToMsg, MakeState
from nav_quadrotor.msg import TrajectoryArray
import sys
sys.path.append("/home/justin/Documents/ivapylibs/control")

from ivacontrol.quadcopter.linQuadCopter import linQuadCopter
from ivacontrol.niODERK4 import niODERK4
from ivacontrol.trajSynth.naive import naive
from ivacontrol.controller.linear import linear
from ivacontrol.structures import structure
from ivacontrol.util import Timer
import numpy as np

import tf2_ros
import tf2_geometry_msgs

#https://stackoverflow.com/a/2891805
np.set_printoptions(precision=3)



class PosesToTrajectories(object):

    def __init__(self):
        self.odom = None

        tf_cache_duration = 10
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # publish to commands
        self.traj_pub = rospy.Publisher("trajectory", data_class=MultiDOFJointTrajectory, queue_size=1)
        self.traj_array_pub = rospy.Publisher("trajectories", data_class=TrajectoryArray, queue_size=1)

        self.sim_pose_array_pub = rospy.Publisher("sim_pose_array", data_class=PoseArray, queue_size=1)
        self.des_pose_array_pub = rospy.Publisher("des_pose_array", data_class=PoseArray, queue_size=1)

        self.ref_pose_array_pub = rospy.Publisher(name="ref_pose_array", data_class=PoseArray, queue_size=1)
        self.adjusted_des_pose_array_pub = rospy.Publisher(name="adjusted_des_pose_array", data_class=PoseArray, queue_size=1)
        self.adjusted_ref_pose_array_pub = rospy.Publisher(name="adjusted_ref_pose_array", data_class=PoseArray, queue_size=1)
        self.interp_pose_array_pub = rospy.Publisher(name="interp_pose_array", data_class=PoseArray, queue_size=1)

        self.setup()

        # subscribe to odometry
        self.odom_sub = rospy.Subscriber("/hummingbird/ground_truth/odometry", Odometry, self.odomCB, queue_size=1)
        self.goal_sub = rospy.Subscriber("/hummingbird/command/pose2", data_class=PoseStamped, callback=self.goalCB, queue_size=1)
        self.goal_poses_sub = rospy.Subscriber("poses", data_class=PoseArray, callback=self.generate_trajectories, queue_size=1)


    def getCurState(self):
        if self.odom is not None:
            xc = MsgToState(odom=self.odom)
            return (xc, self.odom.header)
        return (None, None)

    def setup(self):
        # ==[1] Setup the control system.
        #
        # --[1.1] Define the dynamics of the system.

        Parms = structure()
        Parms.m = .7775752  # kg
        Parms.r = 0.17  # m
        Parms.Jx = 0.007  # kg-m^2
        Parms.Jy = 0.007  # kg-m^2
        Parms.Jz = 0.012  # kg.m^2
        Parms.KT = 0.00000854858  # ???
        Parms.KD = 0.016 * Parms.KT  # ???
        Parms.g = 9.80  # m/s^2
        (linDyn, A, B, uEq) = linQuadCopter.dynamicsAtHoverHummingbird(Parms)

        # --[1.2] Define the trajectory synthesizer.
        #
        tsys = structure()
        tsys.A = A
        tsys.B = B #*math.sqrt(uEqMag)
        tsys.Q = (4e8) * np.diag([4, 4, 9, 4, 4, 1, 3, 3, 10, 7, 7, 3])
        # tsys.Q  = np.diag([200, 300, 400, 100, 10, 10, 20, 20, 10, 10, 10, 10])
        tsys.dt = 0.05
        tsys.solver = niODERK4
        tsys.tspan = [0, 40]
        #tsys.cons.sat.min=0
        #tsys.cons.sat.max=1e6
        self.tsys = tsys

        metaBuilder = structure()
        metaBuilder.regulator = naive.trajBuilder_LinearReg
        metaBuilder.tracker = naive.trajBuilder_LinearTracker

        tMaker = naive(tsys, metaBuilder)
        self.tMaker = tMaker

        self.tracker = naive.trajBuilder_LinearTracker(tsys)

        # --[1.3] Define the trajectory tracking builder.
        #
        csys = structure()
        csys.dt = 0.05
        csys.odeMethod = niODERK4
        csys.controller = linear(np.zeros((12, 1)), 0*uEq)
        #csys.Q = (1e6) * np.diag([2, 2, 6, 4, 4, 1, 3, 3, 10, 9, 9, 3])  # Can be weaker than the tsys.Q
        csys.Q = (1e8) * np.diag([2, 2, 6, 4, 4, 1, 3, 3, 10, 9, 9, 3])  # Can be weaker than the tsys.Q
        self.csys = csys

        # csys.Q  = np.diag([200, 300, 400, 100, 10, 10, 20, 20, 10, 10, 10, 10])
        csys.controller.setByCARE(A, B, csys.Q)

        #print("K: " + str(csys.controller.K))

        self.controller = csys.controller

        #return
        ##All of this other stuff is for forward simulating, which we don't need here, though we could do it in order to compare desired and executed trajectories


        tTracker = csys.controller.structBuilder(linDyn, csys)

        # --[1.4] Pack together into control system.
        #
        cSim = linQuadCopter(linDyn, tMaker, tTracker)
        self.cSim = cSim


    def odomCB(self, odom_msg):
        self.odom = odom_msg

    def goalCB(self, goal_msg):
        goal_pose_array = PoseArray(header=goal_msg.header, poses=[goal_msg.pose])
        self.generate_trajectories(goal_pose_array=goal_pose_array)

    def generate_trajectories(self, goal_pose_array):
        rospy.logdebug("Time since poses generated: " + str((rospy.Time.now() - goal_pose_array.header.stamp).to_sec()) + "s")
        if goal_pose_array.poses == 0:
            rospy.logerr("Empty pose array received!")
            return

        (xc, header) = self.getCurState()
        if xc is not None:
            istate = structure()
            istate.x = xc
            self.cSim.setInitialState(istate)

            def getTransform(target_frame, source_frame, stamp):

                # get the tf at first available time
                try:
                    transformation = self.tf_buffer.lookup_transform(target_frame, source_frame, stamp, rospy.Duration(0.1))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    rospy.logerr('Unable to find the transformation from %s to %s: %s', source_frame, target_frame, str(e))
                    return None
                return transformation

            gen_des_poses = self.des_pose_array_pub.get_num_connections()>0
            gen_ref_poses = self.ref_pose_array_pub.get_num_connections()>0
            gen_traj = self.traj_array_pub.get_num_connections()>0
            gen_interp_poses = self.interp_pose_array_pub.get_num_connections()>0
            gen_sim_poses = self.sim_pose_array_pub.get_num_connections()>0

            v_lin = 0.5

            class EmptyContext(object):
                def __enter__(self):
                    return self

                def __exit__(self, exc_type, exc_value, traceback):
                    return False

            use_timer = True

            def process_trajectory(pose):
                istate = structure(x=xc)

                des_pose_array = None
                ref_pose_array = None
                trajectory = None
                interp_pose_array = None
                sim_pose_array = None

                goal = MsgToState(pose=pose)
                des_traj = ToGoal(xc=xc, goal=goal, v_lin=v_lin)

                des_traj.tspan[1] = min(des_traj.tspan[1], 10)

                if gen_des_poses:
                    with Timer("evaluate analytic destraj") if use_timer else EmptyContext():
                        times = np.arange(start=0, stop=des_traj.tspan[1], step=self.tsys.dt)
                        desired_states = des_traj.x(times)

                    des_pose_array = StateToMsg(msg=PoseArray(), states=desired_states, header=header)

                if gen_ref_poses or gen_traj:
                    with Timer("tracker.simulate") if use_timer else EmptyContext():
                        other = self.tracker(istate=istate, desTraj=des_traj)
                        raw_traj = other.simulate()

                    if gen_ref_poses:
                        ref_pose_array = StateToMsg(msg=PoseArray(), traj=raw_traj, header=header)

                    if gen_traj:
                        trajectory = StateToMsg(msg=MultiDOFJointTrajectory(), traj=raw_traj, header=header)

                if gen_interp_poses:
                    with Timer("tMaker.followPath") if use_timer else EmptyContext():
                        des_sol = self.tMaker.followPath(istate=istate, desTraj=des_traj)
                        times = np.arange(start=0, stop=des_sol.tspan[1], step=self.tsys.dt)
                        desired_states = des_sol.x(times)

                    interp_pose_array = StateToMsg(msg=PoseArray(), states=desired_states, header=header)

                if gen_sim_poses:
                    with Timer("cSim.follow") if use_timer else EmptyContext():
                        sol = self.cSim.follow(des_traj)
                    sim_pose_array = StateToMsg(msg=PoseArray(), traj=sol, header=header)

                return (des_pose_array, ref_pose_array, trajectory, interp_pose_array, sim_pose_array)



            #if self.goal_pose is None:
            #    rospy.logwarn("No goal pose received yet!")

            #Transform poses to world frame
            #Note: this doesn't work!
            transform = getTransform(header.frame_id, goal_pose_array.header.frame_id, rospy.Time(0))  # goal_pose_array.header.stamp)
            if transform is None:
                return

            #transformed_goal_poses = [tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=Pose(position=target_pose.position)), transform).pose for target_pose in goal_pose_array.poses]
            transformed_goal_poses = goal_pose_array.poses
            #for p in transformed_goal_poses:
            #    p.position.z = 1.5

            #for pose in transformed_goal_poses:
            #    (des_pose_array, ref_pose_array, trajectory, interp_pose_array, sim_pose_array) = process_trajectory(pose, v_lin)

            #res = starmap(process_trajectory, zip(transformed_goal_poses, repeat(v_lin)))
            #res = map(process_trajectory, transformed_goal_poses, repeat(v_lin))
            with Timer("Generate " + str(len(transformed_goal_poses)) + " trajectories [Serial]"):
                results = list(map(process_trajectory, transformed_goal_poses))

            if False:
                from pathos.multiprocessing import ProcessingPool as Pool
                from multiprocessing import cpu_count

                with Timer("Generate " + str(len(transformed_goal_poses)) + " trajectories [Parallel]"):
                    #with Pool(processes=10) as pool: # Only available in python 3.3+

                    pool = Pool(processes=10)
                    results = pool.map(process_trajectory, transformed_goal_poses)
                    pool.close()
                    pool.join()

            #https://stackoverflow.com/a/7558990
            (des_pose_array, ref_pose_array, trajectory, interp_pose_array, sim_pose_array) = list(zip(*results))

            #merge where relevant
            def merge_pose_arrays(list_of_arrays):
                def all_poses():
                    for pa in list_of_arrays:
                        for p in pa.poses:
                            yield p

                return PoseArray(header=list_of_arrays[0].header, poses=[p for p in all_poses()])

            if gen_des_poses:
                self.des_pose_array_pub.publish(merge_pose_arrays(des_pose_array))

            if gen_ref_poses:
                self.ref_pose_array_pub.publish(merge_pose_arrays(ref_pose_array))

            if gen_traj:
                self.traj_array_pub.publish(TrajectoryArray(header=trajectory[0].header, trajectories=trajectory))

            if gen_interp_poses:
                self.interp_pose_array_pub.publish(merge_pose_arrays(interp_pose_array))

            if gen_sim_poses:
                self.sim_pose_array_pub.publish(merge_pose_arrays(sim_pose_array))



            #self.cSim.plotSignals(1, desTraj, blocking=False)


    def AnalyzeCircles(self):
        from nav_quadrotor.trajectories import Curve, get_needed_radius
        import matplotlib.pyplot as plt

        v_lin = 0.5
        w = 0

        v_z = 0
        angular_dist = math.pi*2*2

        header=Header(frame_id="world", stamp=rospy.Time.now())



        min_r = 0.35
        max_r = 100
        step_size = 2
        start_r = math.log(min_r, step_size)
        stop_r = math.log(max_r, step_size)
        des_rs = np.logspace(start=start_r, stop=stop_r, base=step_size, endpoint=True, num=10)
        #des_rs = np.array([1])
        #des_rs = np.arange(start=0.1, stop=10, step=0.1)

        actual_rs = np.empty_like(des_rs)
        plt.figure()

        for vx in [0]: #np.arange(start=0, stop=0.5, step=0.1):
            xc = MakeState(vx=vx, wz=w)
            istate = structure(x=xc)

            for i in range(des_rs.size):
                r = des_rs[i]
                desTraj = Curve(xc=xc, v_lin=v_lin, v_z=v_z, r=r)
                tf = old_div(angular_dist, math.fabs(old_div(v_lin, r)))
                tf = min(tf, 20)
                desTraj.tspan = np.array([0, tf])

                other = self.tracker(istate=istate, desTraj=desTraj)
                ref_traj = other.simulate()

                times = np.arange(start=0, stop=desTraj.tspan[1], step=self.tsys.dt)
                desired_states = desTraj.x(times)

                des_end_pos = desired_states[0:2,-1]
                sim_end_pos = ref_traj.x[0:2,-1]

                if True:
                    adj_r = get_needed_radius(r)
                    adjustDesTraj = Curve(xc=xc, v_lin=v_lin, v_z=v_z, r=adj_r)
                    tf = old_div(angular_dist, math.fabs(old_div(v_lin, adj_r)))
                    tf = min(tf, 20)
                    adjustDesTraj.tspan = np.array([0, tf])

                    adj_ref_obj = self.tracker(istate=istate, desTraj=adjustDesTraj)
                    adj_ref_traj = adj_ref_obj.simulate()

                    times = np.arange(start=0, stop=adjustDesTraj.tspan[1], step=self.tsys.dt)
                    adj_desired_states = adjustDesTraj.x(times)


                dist = np.linalg.norm((des_end_pos-sim_end_pos))
                eff_r = r + dist
                actual_rs[i] = eff_r

                print("v_lin=" + str(v_lin) + ", r=" + str(r) + ", des_p=" + str(des_end_pos) + ", sim_p=" + str(sim_end_pos) + ", error=" + str(dist) + ", effective r=" + str(eff_r))

                if self.des_pose_array_pub.get_num_connections()>0:
                    des_pose_array = StateToMsg(msg=PoseArray(), states=desired_states, header=header)
                    self.des_pose_array_pub.publish(des_pose_array)

                if self.ref_pose_array_pub.get_num_connections() > 0:
                    pose_array = StateToMsg(msg=PoseArray(), traj=ref_traj, header=header)
                    self.ref_pose_array_pub.publish(pose_array)

                if self.adjusted_des_pose_array_pub.get_num_connections() > 0:
                    pose_array = StateToMsg(msg=PoseArray(), states=adj_desired_states, header=header)
                    self.adjusted_des_pose_array_pub.publish(pose_array)

                if self.adjusted_ref_pose_array_pub.get_num_connections() > 0:
                    pose_array = StateToMsg(msg=PoseArray(), traj=adj_ref_traj, header=header)
                    self.adjusted_ref_pose_array_pub.publish(pose_array)

                rospy.sleep(0.01)

            #plt.plot(des_rs, actual_rs)
            plt.plot(des_rs, actual_rs-des_rs)
            plt.show(block=False)

        if False:

            plt.figure()
            plt.plot(actual_rs, des_rs, "o", actual_rs, get_needed_radius(actual_rs))
            plt.show(block=True)

        if False:
            print(str(actual_rs))
            print(str(des_rs))

        if False:

            #plt.figure()
            y = des_rs
            x = actual_rs

            weights = old_div(1,y)
            use_limit = False

            for deg in [3,5,7,9]:
                plt.figure()

                coeffs = np.polyfit(x=x, y=y, deg=deg, w=weights)
                poly = np.poly1d(coeffs)
                new_x = x #np.linspace(x[0], x[-1])
                new_y = poly(new_x)
                plt.plot(x, y, "o", new_x, new_y)
                if use_limit:
                    plt.xlim(0,5)
                    plt.ylim(0,6)
                error = np.mean(np.abs(y-new_y))
                print("Deg=" + str(deg) + ", Error: " + str(error))

            print("Now using logs")
            x = actual_rs
            y = actual_rs-des_rs
            weights = np.sqrt(y)
            for deg in [3, 5]: # [3,5,7,9]:
                plt.figure()
                coeffs = np.polyfit(x=x, y=np.log(y), deg=deg) #, w=weights
                poly = np.poly1d(coeffs)
                new_x = x #np.linspace(x[0], x[-1])
                new_y = poly(new_x)
                plt.plot(x, np.log(y), "o", new_x, new_y)
                if use_limit:
                    plt.xlim(0,5)
                error = np.mean(np.abs(y-new_y))
                print("Deg=" + str(deg) + ", Error: " + str(error) + ", coeffs: " + str(coeffs))

                plt.figure()
                plt.plot(x, x-y, "o", new_x, new_x-np.exp(new_y))
                if use_limit:
                    plt.xlim(0,5)
                    plt.ylim(0,6)

            plt.show(block=True)





if __name__ == '__main__':

    try:
        rospy.init_node("poses_to_trajectories")
        rospy.loginfo("poses_to_trajectories node started")
        controller = PosesToTrajectories()

        controller.AnalyzeCircles()

        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.loginfo("Interrupted: " + str(e))

    #except Exception as e:
    #    rospy.logerr("Exception: " + str(e))
