from __future__ import print_function
from __future__ import division

from builtins import str
from builtins import object

from past.utils import old_div

##Standard packages
import math
import sys
import time
import string
from copy import deepcopy

import numpy as np
import cv2
import matplotlib.pyplot as plt
plt.switch_backend('agg')

from scipy.ndimage.morphology import distance_transform_edt
from scipy.interpolate import griddata, interp2d
from sklearn.cluster import DBSCAN
from skimage.morphology import local_minima

##ROS packages
import rospy
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel
from dynamic_reconfigure.server import Server
import rosbag
import tf2_ros
from tf2_geometry_msgs import PoseStamped

##ROS messages
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray

##Custom packages and messages
from egocylindrical import ECCameraModel
from egocylindrical.msg import EgoCylinderPoints
from nav_quadrotor.msg import CandidateWaypoint, CandidateWaypoints, DetailedWaypointArray, WaypointProperty





def plt_to_cv(fig):
    # redraw the canvas
    fig.tight_layout()
    plt.gca().set_aspect('equal')
    plt.gca().invert_yaxis()
    fig.canvas.draw()

    # convert canvas to image
    #img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)

    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

    # img is rgb, convert to opencv's default bgr
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class ImageCostmap(object):

    def __init__(self, image):

        #Note: the image should probably have been set as float32 as soon as loaded from the rosbag
        image = image.astype(dtype=np.float32)
        self.image = image
        shape = image.shape

        self.image_interp = interp2d(x=list(range(shape[1])), y=list(range(shape[0])), z=image, copy=False, bounds_error=True)

        ksize = 5
        self.sobel_x  =cv2.Sobel(src=image, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=ksize, borderType=cv2.BORDER_DEFAULT)
        self.sobel_y = cv2.Sobel(src=image, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=ksize, borderType=cv2.BORDER_DEFAULT)

        #Z = interpolate.griddata((X_table, Y_table), Z_table, (X, Y), method='cubic')
        self.x_interp = interp2d(x=list(range(shape[1])), y=list(range(shape[0])), z = self.sobel_x, copy=False, bounds_error=True)
        self.y_interp = interp2d(x=list(range(shape[1])), y=list(range(shape[0])), z = self.sobel_y, copy=False, bounds_error=True)

        pass

    def f(self, point):
        v = self.image_interp(x=point[0,:], y=point[1,:])
        return v

    def dfdx(self, point):
        dx = self.x_interp(x=point[0,:], y=point[1,:])
        return dx

    def dfdy(self, point):
        dy = self.y_interp(x=point[0,:], y=point[1,:])
        return dy

    def grad(self, point):
        return np.array([self.dfdx(point=point), self.dfdy(point=point)])


def get_distance_transform(image):
    from scipy.ndimage.morphology import distance_transform_edt
    distances = distance_transform_edt(input=image, sampling=None, return_distances=True, return_indices=False)

    return distances


def invert_im(image):
    inverted_im = 0 + (image==0)
    return inverted_im


def plot_levels(image, name):
    if image is None:
        return

    f = plt.figure(num=1, clear=True)
    h = plt.contourf(image, levels=20)
    plt.colorbar(h, orientation='horizontal')
    im = plt_to_cv(f)
    cv2.imshow(name, im)


class GoalCostCalculator(object):

    def __init__(self):
        self.grid_size = None
        self.row_inds = None
        self.col_inds = None

    def get_cost(self, image, goal):
        self.update_size(grid_size=image.shape)
        goal_inds = self.get_goal_inds(goal=goal)
        cost = self.get_goal_distances(goal_inds=goal_inds)
        return cost

    def update_size(self, grid_size):
        if grid_size != self.grid_size:
            rospy.loginfo("New grid_size " + str(grid_size) + " is different from previous " + str(self.grid_size) + ", updating indicies")
            #rows = np.linspace(0, 1, grid_size[0])
            #cols = np.linspace(0, 1, grid_size[1])
            rows = np.arange(start=0, stop=grid_size[0])
            cols = np.arange(start=0, stop=grid_size[1])

            self.row_inds, self.col_inds = np.meshgrid(rows, cols, indexing='ij')
            self.grid_size = grid_size
        else:
            rospy.logdebug("New grid_size " + str(grid_size) + " matches previous " + str(self.grid_size) + ", reusing indicies")

    def get_goal_inds(self, goal):
        goal = np.array(goal)
        goal = goal.astype(dtype=int)
        goal[1] = max(min(goal[1], self.grid_size[0] - 1), 0)
        goal_inds = np.array((goal[1], goal[0]))
        return goal_inds

    def get_goal_distances(self, goal_inds):
        dists = np.sqrt((self.row_inds - goal_inds[0])**2 + (self.col_inds - goal_inds[1])**2)
        return dists


def get_grid_distance(grid_size, goal_inds):
    #TODO: only regenerate these if grid_size changes
    rows = np.linspace(0, 1, grid_size[0])
    cols = np.linspace(0, 1, grid_size[1])
    row_inds, col_inds = np.meshgrid(rows, cols, indexing='ij')
    dr = row_inds - goal_inds[0]
    dc = col_inds - goal_inds[1]
    dists = np.sqrt(dr*dr+dc*dc)
    return dists

def get_goal_cost(accum_conv, goal):
    goal = np.array(goal)
    goal = goal.astype(dtype=int)

    im_shape = accum_conv.shape

    goal[1] = max(min(goal[1], im_shape[0]-1), 0)

    goal_inds = np.array((goal[1], goal[0]))

    goal_cost = get_grid_distance(grid_size=im_shape, goal_inds=goal_inds)
    return goal_cost


def get_masked_im(im, mask):
    masked = im.copy()
    masked[mask>0] = 0
    return masked

def goal_cost_tester(im=None):
    im = np.zeros((200,100)) if im is None else im
    goal = [im.shape[1]/2, im.shape[0]/2]
    goal_cost = get_goal_cost(accum_conv=im, goal=goal)

    plot_levels(image = goal_cost, name="goal_cost_test")

    masked_goal_cost = get_masked_im(im=goal_cost, mask=im)
    plot_levels(image=masked_goal_cost, name="masked_goal_cost_test")


class CostmapApproach(object):
    class CostmapApproachComponent(object):

        #Don't use 'shift'
        def __init__(self, name, image=None, weight=1, thresh=-1, negate=False):
            self.name = name
            self.image = image
            self.weight = weight
            self.thresh = thresh
            self.negate = negate

        def eval(self):
            if self.weight == 0 or self.image is None:
                return None
            else:
                image = self.image.copy()
                if self.thresh >= 0:
                    image[image > self.thresh] = self.thresh

                image *= self.weight

                if self.negate:
                    image *= -1

                return image

        def to_string(self):
            #return self.name + "[w=" + str(self.weight) + ",t=" + str(self.thresh) + "]"
            val = ""
            if not self.weight == 0:
                val += self.name
                open_brackets = False
                if not self.weight==1:
                    val += "[w=" + str(self.weight)
                    open_brackets = True
                if not self.thresh==-1:
                    val += "," if open_brackets else "["
                    open_brackets = True
                    val += "t=" + str(self.thresh)
                if open_brackets:
                    val += "]"
                val += ":"
            return val


    def __init__(self, name, timing = True):
        self.name = name
        self.conv = CostmapApproach.CostmapApproachComponent(name="conv", image=None, weight=0, thresh=-1)
        self.binary_conv = CostmapApproach.CostmapApproachComponent(name="binary_conv", image=None, weight=0, thresh=-1)
        self.ob_edt = CostmapApproach.CostmapApproachComponent(name="ob", image=None, weight=0, thresh=-1)
        self.fs_edt = CostmapApproach.CostmapApproachComponent(name="fs", image=None, weight=0, thresh=-1, negate=True)
        self.goal = CostmapApproach.CostmapApproachComponent(name="goal", image=None, weight=0, thresh=-1)

        self.components = [self.conv, self.binary_conv, self.ob_edt, self.fs_edt, self.goal]

        self.timing = timing
        self.goal_cost_calc = GoalCostCalculator()

    def update(self, im, goal):
        if self.timing:
            start_time = time.time()

        thresh_im = np.zeros_like(im)
        t = 30
        thresh_im[im>t] = im[im>t]

        im = thresh_im
        binary_im = 0 + (im > 0)

        inverted_im = invert_im(image=im)
        #ob_edt = get_distance_transform(image=im)
        fs_edt = get_distance_transform(image=inverted_im)

        self.conv.image = im
        self.binary_conv.image = binary_im
        #self.ob_edt.image = ob_edt
        self.fs_edt.image = fs_edt

        if goal is not None:
            #goal_cost = get_goal_cost(accum_conv=im, goal=goal)
            goal_cost = self.goal_cost_calc.get_cost(image=im, goal=goal)
            max_v = np.max(goal_cost)
            goal_cost -= max_v

            masked_goal_cost = get_masked_im(im=goal_cost, mask=im)
            self.goal.image = masked_goal_cost
        else:
            self.goal_image = None

        if self.timing:
            end_time = time.time()
            rospy.logdebug("Updated CostmapApproach in " + str(end_time -start_time) + "s", logger_name="timing")

    def show_ims(self):
        for component in self.components:
            plot_levels(image=component.eval(), name=self.name + "_" + component.to_string())

        return

    def get_costmap_image(self):
        if self.timing:
            start_time = time.time()

        components = [ce for ce in (c.eval() for c in self.components) if ce is not None ]
        combined = np.sum(components, axis=0)

        if self.timing:
            end_time = time.time()
            rospy.logdebug("Generating costmap image took " + str(end_time -start_time) + "s", logger_name="timing")

        return combined

    def to_string(self):
        return "".join((c.to_string() for c in self.components))


def get_local_minima_inds(image):
    min_inds = local_minima(image=image, indices=True, allow_borders=False)

    return min_inds


class NonMinimalSuppression(object):

    def __init__(self, k, min_dist):
        pass
        self.k = k
        self.min_dist = min_dist

    def get_minima_pnts(self, image):
        rows, cols = get_local_minima_inds(image=image)
        cm_vals = image[rows, cols]

        sorted_inds = np.argsort(cm_vals)
        #sorted_vals = cm_vals[sorted_inds]
        #sorted_rows = rows[sorted_inds]
        #sorted_cols = cols[sorted_inds]

        n = 0
        i = 0
        num_pts = cm_vals.size
        cur_inds = []
        cur_rows = None
        cur_cols = None

        while i < num_pts and n < self.k:
            should_add = False

            cand_ind = sorted_inds[i]
            if len(cur_inds)==0:
                should_add = True
            else:
                cr = rows[cand_ind]
                cc = cols[cand_ind]
                dr = cur_rows - cr
                dc = cur_cols - cc
                dists = dr*dr + dc*dc
                min_d = np.min(dists)
                min_d = math.sqrt(min_d)
                if min_d >= self.min_dist:
                    should_add = True

            if should_add:
                cur_inds.append(cand_ind)
                cur_rows = rows[cur_inds]
                cur_cols = cols[cur_inds]
                n+=1

            i+=1

        pnts = np.vstack((cols[cur_inds], rows[cur_inds]))
        return pnts


class PointSampler(object):
    
    def __init__(self, dists=[1, 2, 3, 4], k=20, min_dist=5, viz=True, timing=True):
        self.dists = dists
        self.goal = None
        self.ca = CostmapApproach(name="minima", timing=timing)
        self.viz = viz
        self.cm_viz =True
        self.timing = timing
        self.camera_model = ECCameraModel()
        self.nms = NonMinimalSuppression(k=k, min_dist=min_dist)

        ca = self.ca
        ca.conv.weight = 0
        ca.binary_conv.weight = 1
        ca.ob_edt.weight = 0
        ca.fs_edt.weight = 1
        ca.fs_edt.thresh = 10
        ca.fs_edt.negate = True
        ca.goal.weight = 0.5

        self.costmap_mat = None

    def update(self, info):
        self.camera_model.set_info(camera_info=info)
        rospy.logdebug("Updating camera model")


    def get_current_goal(self):
        return self.goal

    #Current goal in the camera's coordinate frame
    def set_current_goal(self, goal):
        self.goal = goal

    def get_goal_coords(self):
        goal_pt = self.get_current_goal()
        if goal_pt is None:
            return None, None
        else:
            goal = self.camera_model.project_3d_points_to_pixels(points=goal_pt)
            goal_dist = math.sqrt(goal_pt[0]*goal_pt[0]+goal_pt[2]*goal_pt[2])
            rospy.loginfo("Goal=" + str(goal_pt) + "; goal_dist=" + str(goal_dist) + "; goal_pix=" + str(goal))
            return goal, goal_dist

    def sample_points(self, image, info=None, nearest_depth_im=None):
        if info is not None:
            self.update(info)

        goal_coords, goal_dist = self.get_goal_coords()
        
        if self.viz:
            plot_levels(image=image, name="accum_conv")

        k = sorted(self.dists, reverse=True)[0]

        self.ca.update(im=image, goal=goal_coords)
        if self.viz:
            self.ca.show_ims()
        cm = self.ca.get_costmap_image()

        self.costmap_mat = cm

        if self.timing:
            start_time = time.time()

        pnts = self.nms.get_minima_pnts(image=cm)

        if self.timing:
            end_time = time.time()
            rospy.logdebug("Found local minima in " + str(end_time - start_time) + "s", logger_name="timing")
            start_time = end_time

        if self.viz:
            cv2.imshow("minima_" + str(self.ca.to_string()), cm)
            cv2.waitKey(1)
        if self.timing:
            end_time = time.time()
            rospy.logdebug("Generated costmap_mat " + str(end_time - start_time) + "s", logger_name="timing")
            start_time = end_time

        raw_ppnts = self.camera_model.project_pixels_to_3d_rays(uv=pnts)

        des_dists = nearest_depth_im[pnts[1,:], pnts[0,:]]
        thresh_dists = des_dists if goal_dist is None else np.minimum(des_dists, goal_dist)
        ppnts = raw_ppnts * thresh_dists

        if self.timing:
            end_time = time.time()
            rospy.logdebug("Project & scale image points to 3D" + str(end_time - start_time) + "s", logger_name="timing")

        property_names = ["freespace_dist", "goal_dist", "costmap_value"]
        property_ims = [self.ca.fs_edt.image, self.ca.goal.image, cm]

        wp_data = DetailedWaypointArray()
        wp_data.poses = np_pos_to_pose_msgs(ppnts)
        for pn, pim in zip(property_names,property_ims):
            if pim is not None:
                prop = WaypointProperty()
                prop.name = pn
                prop.values = pim[pnts[1,:], pnts[0,:]]
                wp_data.properties.append(prop)

        return ppnts, wp_data


class CostmapVisualizer(object):

    ##TODO: make it a lazy subscriber
    def __init__(self, crop_image):
        #self.show_fig = True
        self.crop_image = crop_image
        self.timing = True

        self.camera_model = ECCameraModel()
        self.cv_bridge = CvBridge()

        self.cm_viz_pub = rospy.Publisher("costmap_image", Image, latch=True, queue_size=1)

        self.image_sub = message_filters.Subscriber("costmap_image_raw", Image, queue_size=1)
        self.info_sub = message_filters.Subscriber("camera_info", EgoCylinderPoints, queue_size = 1)
        self.point_sub = message_filters.Subscriber("sampled_points", PoseArray,  queue_size=1)
        self.info_cache = message_filters.Cache(self.info_sub, cache_size=30)

        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.point_sub], 3)
        self.ts.registerCallback(self.msg_cb)


    def msg_cb(self, costmap_msg, minima_msg):
        rospy.logdebug("Message callback")
        if self.cm_viz_pub.get_num_connections()==0:
            return

        info_msg = self.get_info_msg(costmap_msg.header.stamp)

        if info_msg is not None:
            self.process_costmap(costmap_msg=costmap_msg, info_msg=info_msg, minima_msg=minima_msg)


    def process_costmap(self, costmap_msg, info_msg, minima_msg):
        cm, pnts = self.convert_msgs(costmap_msg=costmap_msg, info_msg=info_msg, minima_msg=minima_msg)
        costmap_mat = self.generate_costmap_visual(cm=cm, pnts=pnts)

        self.publish_visual(costmap_mat=costmap_mat, header=costmap_msg.header)


    def convert_msgs(self, costmap_msg, info_msg, minima_msg):
        if self.timing:
            start_time = time.time()

        cm = self.cv_bridge.imgmsg_to_cv2(costmap_msg, desired_encoding='passthrough')
        if self.timing:
            end_time = time.time()
            rospy.logdebug("Converted image msgs to cv in" + str(end_time - start_time) + "s", logger_name="timing")
            start_time = end_time

        rospy.logdebug("Updating camera model")
        self.camera_model.set_info(camera_info=info_msg)

        ppnts = np.array(([p.position.x for p in minima_msg.poses], [p.position.y for p in minima_msg.poses], [p.position.z for p in minima_msg.poses]))
        pnts = self.camera_model.project_3d_points_to_pixels(ppnts)
        if self.timing:
            end_time = time.time()
            rospy.logdebug("Converted pose_array to numpy array in" + str(end_time - start_time) + "s", logger_name="timing")

        return cm, pnts


    def generate_costmap_visual(self, cm, pnts):
        if self.timing:
            start_time = time.time()
        fig = plt.figure(num=1, clear=True)
        h = plt.contourf(cm, levels=20)
        plt.colorbar(h, orientation='horizontal')

        if self.timing:
            end_time = time.time()
            rospy.logdebug("Generated costmap figure " + str(end_time - start_time) + "s", logger_name="timing")
            start_time = end_time

        #plt.scatter(x=pnts[0], y=pnts[1], s=4, color='red')
        if self.timing:
            end_time = time.time()
            rospy.logdebug("Plotted local minima in " + str(end_time - start_time) + "s", logger_name="timing")
            start_time = end_time

        costmap_mat = plt_to_cv(fig=fig)
        if self.timing:
            end_time = time.time()
            rospy.logdebug("Generated costmap_mat " + str(end_time - start_time) + "s", logger_name="timing")

        relevant_image = costmap_mat[190:331, 51:618] if self.crop_image else costmap_mat

        return relevant_image


    def publish_visual(self, costmap_mat, header):
        if self.cm_viz_pub.get_num_connections() > 0:
            start_time = time.time()
            cm_msg = self.cv_bridge.cv2_to_imgmsg(costmap_mat, encoding="bgr8")
            cm_msg.header = header
            self.cm_viz_pub.publish(cm_msg)
            if self.timing:
                end_time = time.time()
                rospy.logdebug("Converted and published costmap image in " + str(end_time - start_time) + "s", logger_name="timing")


    def get_info_msg(self, stamp):
        interval = self.info_cache.getInterval(from_stamp=stamp, to_stamp=stamp)
        if len(interval) == 0:
            return None
        else:
            return interval[0]



class GoalTransformer(object):

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.fixed_frame_id = "world"
        self.goal = None
        self.path = None
        self.look_ahead_dist = 2
        self.cur_goal_pub = rospy.Publisher("cur_path_goal", PoseStamped, queue_size=1, latch=True)
        self.cur_goal_pub2 = rospy.Publisher("cur_path_goal_pose_array", PoseArray, queue_size=1, latch=True)

    def set_goal(self, goal):
        rospy.loginfo("GoalTransformer received new goal!", logger_name="goal_transformer.set_goal")
        self.goal = goal
        if self.path is not None and self.path.header.stamp < goal.header.stamp:
            self.path = None

    def set_path(self, path):
        rospy.loginfo("GoalTransformer received new path!", logger_name="goal_transformer.set_path")
        self.path = path

    #Transform goal into frame/stamp of the provided header
    def get_goal(self, header):
        goal = self.goal
        if goal is None:
            return None

        if self.path is not None:
            path = self.path

            try:
                camera_pose_in_world = self.tfBuffer.transform_full(object_stamped=PoseStamped(header=header, pose=Pose(orientation=Quaternion(w=1))), target_frame=path.header.frame_id, target_time=path.header.stamp, fixed_frame=self.fixed_frame_id)

                tf = self.tfBuffer.lookup_transform_full(target_frame=header.frame_id, target_time=header.stamp, source_frame=path.header.frame_id, source_time=path.header.stamp, fixed_frame=self.fixed_frame_id)
            except tf2_ros.TransformException as e:
                rospy.logwarn("Error transforming camera to world frame or getting world to camera transform:\n" + str(e))
                return None

            do_transform = self.tfBuffer.registration.get(type(PoseStamped()))
            transform_pose = lambda p: do_transform(p, tf)

            np_path = np.zeros((2,len(path.poses)))
            for pose_ind in range(len(path.poses)):
                p = path.poses[pose_ind].pose.position
                np_path[0, pose_ind] = p.x
                np_path[1, pose_ind] = p.y

            np_cam = np.array([camera_pose_in_world.pose.position.x, camera_pose_in_world.pose.position.y]).reshape((2,1))
            path_dists = np.linalg.norm(np_path - np_cam, axis=0)
            nearest_pose_ind = np.argmin(path_dists)

            inter_pose_dists = np.linalg.norm(np_path[:,nearest_pose_ind+1:]-np_path[:,nearest_pose_ind:-1], axis=0)
            cand_path_dists = path_dists[nearest_pose_ind:]

            best_pose_ind = nearest_pose_ind
            integrated_dist = 0
            for cand_ind in range(inter_pose_dists.size):
                p_dist = cand_path_dists[cand_ind]
                integrated_dist += inter_pose_dists[cand_ind]

                #rospy.logdebug("Pose=" + str(pose) + "\n\n dist=" + str(p_dist) + ", path_length=" + str(integrated_dist))
                if p_dist < self.look_ahead_dist and integrated_dist < self.look_ahead_dist:
                    best_pose_ind = cand_ind + nearest_pose_ind
                    #rospy.logdebug("New best pose: \n" + str(pose))
                else:
                    break

            best_pose = path.poses[best_pose_ind]
            rospy.loginfo("Best pose = \n" + str(best_pose))

            if best_pose is None:
                return None

            best_pose.pose.position.z = goal.pose.position.z
            tg = transform_pose(p=best_pose)

        else:
            try:
                tg = self.tfBuffer.transform_full(object_stamped = goal, target_frame=header.frame_id, target_time=header.stamp, fixed_frame=self.fixed_frame_id)
            except tf2_ros.TransformException as e:
                rospy.logwarn("Error transforming goal pose:\n" + str(e))
                return None

        np_goal = np.array([tg.pose.position.x,tg.pose.position.y,tg.pose.position.z])
        rospy.logdebug(
            "GoalTransformer transformed goal from " + str(goal) + " to " + str(tg) + "; goal = " + str(np_goal),
            logger_name="goal_transformer.get_goal")

        tgpa = PoseArray(header=tg.header, poses=[tg.pose])

        self.cur_goal_pub.publish(tg)
        self.cur_goal_pub2.publish(tgpa)

        return np_goal


def np_pos_to_pose_msgs(pos):
    return [Pose(position=Point(x=pos[0,i], y=pos[1,i], z=pos[2,i]), orientation=Quaternion(w=1)) for i in range(pos.shape[1])]


class PointSamplingNode(object):


    def __init__(self, viz=True, timing=True, profiler=None):
        #TODO: use rosparam to get and set parameters
        self.viz = viz
        self.goal_transformer = GoalTransformer()

        dists = rospy.get_param('~analysis_ranges', [1,2,3])
        k = rospy.get_param('~non_min_suppression/k', 20)
        min_dist = rospy.get_param('~non_min_suppression/min_dist', 5)

        rospy.loginfo("Analysis ranges=" + str(dists))

        self.sampler = PointSampler(dists=dists, k=k, min_dist=min_dist, viz=viz, timing=timing)
        self.timing = timing
        self.profiling = (profiler is not None)

        if self.profiling:
            self.pr = profiler

        self.cv_bridge = CvBridge()

        image_topic = "/hummingbird/freespace_estimator_node/freespace_estimator/convolved_projection"
        info_topic = "/egocylinder/egocylinder_info"
        goal_topic = "/nav_goal"
        path_topic = "/hummingbird/obstacle_avoidance_controller_node/potentials_planner/plan"

        nearest_depth_topic = "/egocylinder/nearest_depth_im"

        #TODO: delay starting image subscribers until after a global goal/path has been received
        self.image_sub = message_filters.Subscriber(image_topic, Image, queue_size = 1)
        self.info_sub = message_filters.Subscriber(info_topic, EgoCylinderPoints, queue_size = 1)
        self.nearest_depth_sub = message_filters.Subscriber(nearest_depth_topic, Image, queue_size = 1)
        self.goal_sub = rospy.Subscriber(goal_topic, PoseStamped, self.goal_transformer.set_goal, queue_size=1)
        self.path_sub = rospy.Subscriber(path_topic, Path, self.goal_transformer.set_path, queue_size=1)

        self.point_pub = rospy.Publisher("sampled_points", PoseArray, latch=True, queue_size=2)
        self.cm_img_pub = rospy.Publisher("costmap_image_raw", Image, latch=True, queue_size=1)
        #self.candidate_waypoints_pub = rospy.Publisher("detailed_waypoints", CandidateWaypoints, latch=True, queue_size=2)
        self.gap_sizes_markers_pub = rospy.Publisher("gap_sizes_markers", MarkerArray, latch=True, queue_size=2)
        self.detailed_waypoints_pub = rospy.Publisher("detailed_waypoints", DetailedWaypointArray, latch=True, queue_size=2)

        synchronized = True

        if synchronized:
            self.ts = message_filters.TimeSynchronizer([self.image_sub, self.info_sub, self.nearest_depth_sub], 2)
            self.ts.registerCallback(self.image_callback)
        else:
            self.image_sub.registerCallback(self.image_callback)
            self.info_sub.registerCallback(self.sampler.update)

    def image_callback(self, image, info=None, nearest_depth_im=None):
        if self.profiling:
            self.pr.enable()

        if info is not None:
            rospy.logdebug("Syncronized messages received!", logger_name="message_cb")
        else:
            rospy.logdebug("Image message received!", logger_name="message_cb")

        start_time = time.time()
        cv_image = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        cv_nearest_depth = self.cv_bridge.imgmsg_to_cv2(nearest_depth_im, "passthrough")
        if self.timing:
            next_time = time.time()
            rospy.logdebug("Converted image msgs to cv in" + str(next_time - start_time) + "s", logger_name="timing")


        goal = self.goal_transformer.get_goal(header=image.header)
        self.sampler.set_current_goal(goal=goal)
        if self.timing:
            ttime = time.time()
            rospy.logdebug("Got current goal in " + str(ttime - next_time) + "s", logger_name="timing")
            next_time = ttime

        cm_viz = (self.cm_img_pub.get_num_connections() > 0)
        ppnts, wp_data = self.sampler.sample_points(image=cv_image, info=info, nearest_depth_im=cv_nearest_depth)
        wp_data.header = image.header
        if self.timing:
            ttime = time.time()
            rospy.logdebug("Sampled points in " + str(ttime - next_time) + "s", logger_name="timing")
            next_time = ttime

        #poses = np_pos_to_pose_msgs(pos=ppnts)
        pose_array = PoseArray(header=image.header, poses=wp_data.poses)
        self.point_pub.publish(pose_array)
        if self.timing:
            ttime = time.time()
            rospy.logdebug("Published pose_array in " + str(ttime - next_time) + "s", logger_name="timing")
            next_time = ttime

        if cm_viz and self.sampler.costmap_mat is not None:
            cm_msg = self.cv_bridge.cv2_to_imgmsg(self.sampler.costmap_mat, encoding="passthrough")
            cm_msg.header = image.header
            self.cm_img_pub.publish(cm_msg)
            if self.timing:
                ttime = time.time()
                rospy.logdebug("Converted and published costmap image in " + str(ttime - next_time) + "s", logger_name="timing")
                next_time = ttime

        def get_candidate_waypoints_msg():
            cws = CandidateWaypoints()

            for (p, fs) in zip(poses, fs_vals):
                cw = CandidateWaypoint()
                cw.pose.header = image.header
                cw.pose.pose = p
                cw.freespace_dist = fs
                cws.header = image.header
                cws.waypoints.append(cw)

            return cws

        #self.candidate_waypoints_pub.publish(get_candidate_waypoints_msg())
        self.detailed_waypoints_pub.publish(wp_data)

        #TODO: possibly move this to separate node that subscribes to detailed candidates topic
        def get_gap_size_markers_msg():
            markers = MarkerArray()

            marker = Marker()
            marker.header = image.header
            marker.action = Marker.ADD
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.z = 0.1

            gap_color = ColorRGBA(a=1, r=1)
            marker.color = gap_color

            for prop in wp_data.properties:
                pn = prop.name
                vals = prop.values

                marker.ns = pn
                marker.id = 0

                for (p, fs) in zip(wp_data.poses, vals):
                    my_marker = deepcopy(marker)
                    my_marker.pose = p
                    my_marker.pose.position.y -= 0.1
                    my_marker.text = str(int(fs))
                    markers.markers.append(my_marker)

                    marker.id += 1

            return markers

        self.gap_sizes_markers_pub.publish(get_gap_size_markers_msg())


        if self.viz:
            print(ppnts)

        if self.timing:
            end_time = time.time()
            rospy.loginfo("Waypoint sampling took " + str(end_time - start_time) + "s", logger_name="timing")
        if self.profiling:
            self.pr.disable()

