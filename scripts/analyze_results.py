import os.path

import rospy
import rosbag
import rospy
from nav_scripts.result_analyzer import ResultAnalyzer, getValuesForKeys, filter as filter_results
import pandas as pd

class LogParsingInterface(object):
    pass

class Measurement(object):
    unit_scales = {"ns": 1, "us": 1000, "s": 1e9, "ms": 1e6}

    @staticmethod
    def get_scale_factor(unit):
        return Measurement.unit_scales[unit]

    @staticmethod
    def get_conversion_factor(source_unit, target_unit):
        return Measurement.get_scale_factor(source_unit) / Measurement.get_scale_factor(target_unit)


    def __init__(self, values, unit):
        self.values = values
        self.unit = unit

    def convert_to(self, unit):
        scaled_values = self.values * Measurement.get_conversion_factor(source_unit=self.unit, target_unit=unit)
        return Measurement(values=scaled_values, unit=unit)

class LogMeasurementDefinition(object):

    def __init__(self, name, location, prestring="", unit="s", node=None, datatype=float):
        res = location.split(":")
        num_parts = len(res)
        #if num_parts < 2 or num_parts > 5:
        #    raise RuntimeError("Badly formatted location string: " + str(location))
        self.file = res[0]

        self.function = ":".join(res[1:-1])
        self.line = res[-1]
        if False:
            if len(res[2])==0:
                file_type = "cpp"
            else:
                file_type = "python"

            #split_tup = os.path.splitext(self.file)
            #extension = split_tup[1]
            #file_type = None
            #if extension == "py":
            #    file_type = "python"
            #elif extension in ["cpp", "h", "hpp"]:
            #    file_type = "cpp"


            if file_type == "python":
                self.function = res[1]
                self.line = res[2] if num_parts == 3 else None
            elif file_type == "cpp":
                self.function = res[1] + "::" + res[3]
                self.class_name = res[1]
                self.line = res[4] if num_parts == 5 else None
            else:
                raise RuntimeError("Could not interpret location string: " + str(location))


        self.name=name
        self.prestring=prestring
        self.unit=unit
        self.node=node
        self.datatype=datatype

    def get_signature(self):
        return self.file + self.function

def get_measurement_definitions():
    meas = [
        #Measurement(location="/home/justin/catkin_ws/src/egocylindrical/src/egocylindrical_propagator.cpp:EgoCylindricalPropagator::update:138", prestring="Adding depth image took ", unit="ms"),
        LogMeasurementDefinition(name="ec_update", location="/home/justin/catkin_ws/src/egocylindrical/src/egocylindrical_propagator.cpp:EgoCylindricalPropagator::update:193", prestring="Total time: ", unit="ms"),
        LogMeasurementDefinition(name="ec_2_im", node="/egocylinder/egocylindrical_to_range_image_nodelet", location="/home/justin/catkin_ws/src/egocylindrical/src/range_image_generator.cpp:EgoCylinderRangeImageGenerator::ecPointsCB:106", prestring="Generating egocylindrical range image took ", unit="ms"),

        LogMeasurementDefinition(name="cons_can_infl", node="/egocylinder/conservative_can_image_inflator", location="/home/justin/catkin_ws/src/egocylindrical/src/can_image_inflator_generator.cpp:CanImageInflatorGenerator::imgCB:329", prestring="Inflating egocan lid image by {0.32x0.23} took ", unit="ms"),
        LogMeasurementDefinition(name="cons_img_infl", node="/egocylinder/conservative_range_image_inflator_nodelet", location="/home/justin/catkin_ws/src/egocylindrical/src/range_image_inflator_generator.cpp:RangeImageInflatorGenerator::imgCB:407", prestring="Inflating range image by {0.32x0.23} took ", unit="ms"),

        LogMeasurementDefinition(name="lib_can_infl", node="/egocylinder/liberal_can_image_inflator", location="/home/justin/catkin_ws/src/egocylindrical/src/can_image_inflator_generator.cpp:CanImageInflatorGenerator::imgCB:329", prestring="Inflating egocan lid image by {0.04x0.23} took ", unit="ms"),
        LogMeasurementDefinition(name="lib_img_infl", node="/egocylinder/liberal_range_image_inflator_nodelet", location="/home/justin/catkin_ws/src/egocylindrical/src/range_image_inflator_generator.cpp:RangeImageInflatorGenerator::imgCB:407", prestring="Inflating range image by {0.04x0.23} took ", unit="ms"),

        LogMeasurementDefinition(name="cm_sample", location="endpoint_sampling.py:PointSamplingNode.image_callback:2231", prestring="Waypoint sampling took ", unit="s"),

        LogMeasurementDefinition(name="proj_conv", location="/home/justin/catkin_ws/src/nav_quadrotor/src/freespace_estimator.cpp:update:623", prestring="Total processing time: ", unit="ms"),
        LogMeasurementDefinition(name="fastmarch",
                                 location="/home/justin/catkin_ws/src/nav_quadrotor/src/freespace_estimator.cpp:update:596",
                                 prestring="Fastmarch time: ", unit="ms"),
        LogMeasurementDefinition(name="nd_fm", location="/home/justin/catkin_ws/src/nav_quadrotor/src/freespace_estimator.cpp:update:617", prestring="Nearest Depths processing time: ", unit="ms"),

        LogMeasurementDefinition(name="total_planning", location="/home/justin/catkin_ws/src/trajectory_based_nav/include/trajectory_based_nav/general_nav_impl.h:PlanningData trajectory_based_nav::GeneralNavImpl<T>::Plan:405", prestring="Total: ", unit="ms"),
        LogMeasurementDefinition(name="just_checking",
                                 location="/home/justin/catkin_ws/src/trajectory_based_nav/include/trajectory_based_nav/general_nav_impl.h:PlanningData trajectory_based_nav::GeneralNavImpl<T>::Plan:412",
                                 prestring="STATISTICS: {\"total_planning_time\"", unit="us"),
        LogMeasurementDefinition(name="", location="/home/justin/catkin_ws/src/nav_quadrotor/src/multi_level_trajectory_verifier.cpp:MultiLevelTrajectoryVerifier::collisionCheckPose:133")
    ]
    return meas

from collections import defaultdict
import re
import numpy as np

class LoggingParser(object):
    def __init__(self, definitions):
        self.line_ll = defaultdict(list)

        for d in definitions:
            self.line_ll[d.get_signature()].append(d)

        self.definitions = {d.name : d for d in definitions}
        self.measurements = defaultdict(list)

        numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
        self.rx = re.compile(numeric_const_pattern, re.VERBOSE)

    def add_msg(self, msg):
        def get_signature(m):
            return m.file + m.function

        matching_lines = self.line_ll[get_signature(msg)]
        for ml in matching_lines:
            if (ml.node is not None and ml.node !=msg.name): #ml.file != msg.file or
                continue
            else:
                meas_list = self.measurements[ml.name]

                #Parse out the value of interest
                match = self.rx.search(string=msg.msg, pos=len(ml.prestring))
                if match:
                    startp, endp = match.span()
                    m = match.string[startp:endp]
                    #print(ml.name + ": " + m)
                    v = ml.datatype(m)
                    meas_list.append(v)
                    pass

                    return True
                else:
                    continue


        return False

import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

class TimingLoader(object):

    def __init__(self):
        patterns = get_measurement_definitions()
        self.parser = LoggingParser2(definitions=patterns)



    def load_from_file(self, file):
        with rosbag.Bag(f=file, mode="r") as bag:
            self.load_from_bag(bag=bag)

    def load_from_bag(self, bag):
        c = 0
        w = 0
        for topic, msg, t in bag.read_messages(topics=['/rosout_agg']):
            #print(msg.msg)
            if self.parser.add_msg(msg=msg):
                w+=1
            c+=1
            if c % 500==0:
                rospy.loginfo("Processed " + str(w) +"/" + str(c) + " log statements...")

    def make_graphs(self):
        for k,v in self.parser.measurements.items():
            array_vals = np.array(v)
            avg = np.mean(array_vals)
            print(k + ": mean=" + str(avg) + self.parser.definitions[k].unit)

            plt.figure()
            plt.hist(x=array_vals)
            plt.title(k)

            pass
        plt.show()


class CollisionFinder(object):

    def __init__(self, log_details=False):
        self.max_dt = rospy.Duration(10)
        self.collision_starts = []
        self.log_details = log_details

    def analyze_file(self, file):
        with rosbag.Bag(f=file, mode="r") as bag:
            self.analyze_bag(bag=bag)

    def analyze_bag(self, bag):
        c = 0
        w = 0
        last_collision = None
        for topic, msg, t in bag.read_messages(topics=['/hummingbird/bumper']):
            if len(msg.states)>0:
                w +=1
                mt = msg.header.stamp

                if last_collision is None or mt - last_collision > self.max_dt:
                    print("Found new collision at t=" + str(mt))
                    self.collision_starts.append(mt)
                else:
                    if self.log_details:
                        print("Found continuing collision at t=" + str(mt))

                last_collision = mt


            c+=1
            if c % 5000==0:
                if self.log_details:
                    print("Found " + str(w) + " collisions in " + str(c) + " contact messages")




class RunStartTimesFinder(object):

    def __init__(self):
        self.start_times = []
        self.offset_time = None

    def analyze_file(self, file):
        with rosbag.Bag(f=file, mode="r") as bag:
            self.analyze_bag(bag=bag)

    def analyze_bag(self, bag):
        bag_start_time = bag.get_start_time()
        self.offset_time = bag_start_time
        for topic, msg, t in bag.read_messages(topics=['/hummingbird/obstacle_avoidance_controller_node/potentials_planner/plan']):
            plan_time = msg.header.stamp.to_sec()
            self.start_times.append(msg.header.stamp)
            start_time = plan_time - bag_start_time
            print(str(start_time) + " (" + str(msg.header.stamp) + ")[" + str(t) + "]")




#TODO: combine all of them into 1 to make use of each's information
#First, the result analyzer to get the type/seed/etc of each run
#Then, only go through the rosbag once, using the topic to lookup the appropriate handler
#The goal is to associate results (collisions, statistics, etc) with actual runs (it would probably be better in the future
#to automatically start separate rosbag recorders for each run for simplicity)
#Also, it would likely be good to run a separate node that listens to the contacts topic and publishes events when they occur
#to limit the number of messages that have to be stored


class RosbagTopicAnalyzer(object):

    def __init__(self, topic):
        self.topic = topic

    def process(self, topic, msg, t):
        pass



class PlanningTimesAnalyzer(RosbagTopicAnalyzer):

    def __init__(self, results):
        super(PlanningTimesAnalyzer, self).__init__('/hummingbird/obstacle_avoidance_controller_node/potentials_planner/plan')
        self.results = results
        self.current_index = None
        self.current_result = None
        #self.start_times

    def process(self, topic, msg, t):
        #plan_time = msg.header.stamp.to_sec()
        #self.start_times.append(msg.header.stamp)

        if len(msg.poses)==0:
            print("Skipping empty planning message at time " + str(msg.header.stamp))
            return

        if self.current_index is None:
            self.current_index = 0
        else:
            self.current_index += 1

        self.current_result = self.results[self.current_index]
        self.current_result["extra_stuff"] = {'planning_time': msg.header.stamp}

        print("Planning for run #" + str(self.current_index) + " found at " + str(msg.header.stamp) + "...")

    def get_current_result(self):
        return self.current_result["extra_stuff"] if self.current_result is not None else None


class CollisionData(object):
    def __init__(self):
        # self.c=0
        # self.w=0
        self.last_collision = None
        self.collisions = []
        self.points = []

class CollisionTopicAnalyzer(RosbagTopicAnalyzer):

    def __init__(self, planning_times_analyzer, log_details=False):
        super(CollisionTopicAnalyzer, self).__init__('/hummingbird/bumper')
        self.max_dt = rospy.Duration(10)
        self.collision_starts = []
        self.log_details = log_details
        self.pta = planning_times_analyzer

        self.c = 0
        self.w = 0
        last_collision = None


    def process(self, topic, msg, t):
        if len(msg.states)>0:
            data = self.pta.get_current_result()
            if data is None:
                if self.log_details:
                    print("Found collision before the start of planning, at t=" + str(msg.header.stamp))
                return



            if "collisions" not in data:
                cd = CollisionData()
                data["collisions"] = cd
            else:
                cd = data["collisions"]


            self.w +=1
            mt = msg.header.stamp

            if cd.last_collision is None or mt - cd.last_collision > self.max_dt:
                print("Found new collision at t=" + str(mt))
                self.collision_starts.append(mt)
            else:
                if self.log_details:
                    print("Found continuing collision at t=" + str(mt))

            cpt = msg.states[0].contact_positions[0]
            pt = np.array([cpt.x, cpt.y, cpt.z])

            cd.collisions.append(mt)
            cd.points.append(pt)
            cd.last_collision = mt


        self.c+=1
        if self.c % 5000==0:
            if self.log_details:
                print("Found " + str(self.w) + " collisions in " + str(self.c) + " contact messages")



class OdometryTopicAnalyzer(RosbagTopicAnalyzer):

    def __init__(self, planning_times_analyzer, log_details=False):
        super(OdometryTopicAnalyzer, self).__init__('/hummingbird/ground_truth/odometry')
        self.log_details = log_details
        self.pta = planning_times_analyzer
        self.c = 0


    def process(self, topic, msg, t):
        data = self.pta.get_current_result()
        if data is None:
            if self.log_details:
                print("Found collision before the start of planning, at t=" + str(msg.header.stamp))
            return

        pos = msg.pose.pose.position
        pt = np.array([pos.x, pos.y, pos.z])
        data["last_pos"] = pt

        self.c+=1
        if self.c % 5000==0:
            if self.log_details:
                print("Found " + str(self.c) + " odometry msgs")



class LoggingParser2(object):
    def __init__(self, definitions):
        self.line_ll = defaultdict(list)

        for d in definitions:
            self.line_ll[d.get_signature()].append(d)

        self.definitions = {d.name : d for d in definitions}

        numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
        self.rx = re.compile(numeric_const_pattern, re.VERBOSE)

    def add_msg(self, msg):
        def get_signature(m):
            return m.file + m.function

        #if msg.line == '412' or '"planned":false' in msg.msg:
        #    pass
        #    print("Might be target")

        #Get list of definitions matching msg
        definition_signature_matches = self.line_ll[get_signature(msg)]
        search_matches = {}

        for mdef in definition_signature_matches:
            if mdef.node is not None and mdef.node !=msg.name:
                continue
            else:
                #meas_list = self.measurements[mdef.name]

                # Parse out the value of interest
                # Parse out the value of interest
                if msg.msg.startswith(mdef.prestring):
                    match = self.rx.search(string=msg.msg, pos=len(mdef.prestring))
                    if match:
                        search_matches[mdef] = match

        num_rx_matches = len(search_matches.keys())

        if num_rx_matches > 0:
            candidate_match = None
            selected_match = None
            for (mdef, match) in search_matches.items():
                if candidate_match is None:
                    candidate_match = (mdef, match)
                if str(msg.line) == mdef.line:
                    selected_match = (mdef, match)
                    break

                if num_rx_matches == 1:
                    selected_match = candidate_match

            if selected_match is None:
                print("Warning, multiple possible matches found! Using the first")
                selected_match = candidate_match

            sel_def, sel_match = selected_match
            startp, endp = sel_match.span()
            m = sel_match.string[startp:endp]
            # print(ml.name + ": " + m)
            v = sel_def.datatype(m)
            # meas_list = measurements[ml.name]
            # meas_list.append(v)
            pass

            return sel_def.name, v, msg.header.stamp





        return False, False, False


def get_measurement_dict():
    definitions = get_measurement_definitions()
    res = {d.name : d for d in definitions}
    return res

class StatisticsAnalyzer(RosbagTopicAnalyzer):

    def __init__(self, planning_times_analyzer, log_details=True):
        super(StatisticsAnalyzer, self).__init__('/rosout_agg')
        self.pta = planning_times_analyzer

        patterns = get_measurement_definitions()
        self.parser = LoggingParser2(definitions=patterns)

        self.c = 0
        self.w = 0
        self.log_details = log_details

    def process(self, topic, msg, t):
        #print(msg.msg)

        meas_name, meas_value, meas_time = self.parser.add_msg(msg=msg)
        if meas_name:
            data = self.pta.get_current_result()
            if data is None:
                if self.log_details:
                    print("Ignoring measurement from before start of planning, at t=" + str(t))
                return

            if "statistics" not in data:
                measurements = defaultdict(list)
                data["statistics"] = measurements
            else:
                measurements = data["statistics"]
            measurements[meas_name].append(meas_value)
            measurements[meas_name+"_time"].append(meas_time)

            self.w+=1

        self.c+=1

        if self.c % 20000==0:
            rospy.loginfo("Processed " + str(self.w) +"/" + str(self.c) + " log statements...")

    def make_graphs(self, measurements):
        for k,v in measurements.items():
            array_vals = np.array(v)
            avg = np.mean(array_vals)
            print(k + ": mean=" + str(avg) + self.parser.definitions[k].unit)

            plt.figure()
            plt.hist(x=array_vals)
            plt.title(k)

            pass
        plt.show()


class CombinedResultAnalyzer(object):

    def __init__(self):
        pass


    def analyze(self, result_file=None, bag_file=None, results=None, log_details=True):
        if result_file is not None:
            analyzer = ResultAnalyzer()
            analyzer.readFiles(filenames=result_file, replacements={'repeat':'seed'})
            self.results = analyzer.getCases()
        elif results is not None:
            self.results = results

        if bag_file is not None:
            print(bag_file)
            self.analyze_file(file=bag_file, log_details=log_details)


    def analyze_file(self, file, log_details=True):
        with rosbag.Bag(f=file, mode="r") as bag:
            self.analyze_bag(bag=bag, log_details=log_details)

    def analyze_bag(self, bag, log_details=True):
        pta = PlanningTimesAnalyzer(results=self.results)
        ca = CollisionTopicAnalyzer(planning_times_analyzer=pta, log_details=log_details)
        sa = StatisticsAnalyzer(planning_times_analyzer=pta, log_details=log_details)
        oa = OdometryTopicAnalyzer(planning_times_analyzer=pta, log_details=log_details)

        analyzers = [pta, ca, sa, oa]

        topic_list = [a.topic for a in analyzers]
        topic_map = {a.topic: a.process for a in analyzers}

        for topic, msg, t in bag.read_messages(topics=topic_list):
            process_func = topic_map[topic]
            process_func(topic, msg, t)


    def export_data(self, filepath):
        import pickle

        print("Exporting data to: " + str(filepath))
        try:
            file = open(file=filepath, mode='wb')
        except FileNotFoundError as e:
            print(str(e))
            a = 1
        pickle.dump(self.results, file=file)

    def load_data(self, filepath):
        import pickle

        print("Loading data from: " + str(filepath))
        file = open(file=filepath, mode='rb')
        self.results= pickle.load(file=file)

    def process_data(self):
        #analyzer = ResultAnalyzer()
        #analyzer.results = self.results

        #results = analyzer.getCases(whitelist={})

        results_w_planning = []
        for result in self.results:
            good_result=False
            try:
                if 'extra_stuff' in result:
                    data = result['extra_stuff']
                    good_result = True

                    if 'statistics' in data:
                        stats = data['statistics']
                        #df = pd.DataFrame(stats)

                        pt = stats['total_planning_time']
                        ptns = np.array([t.to_nsec() for t in pt])
                        stats['total_planning_period'] = ptns[1:]-ptns[:-1]

                        ct = np.array(stats['just_checking_time'])
                        ctns = np.array([t.to_nsec() for t in ct])
                        stats['just_checking_period'] = ctns[1:]-ctns[:-1]
                        #result['data'] = df

                    if 'collisions' in data:
                        c = data["collisions"]
                        if len(c.collisions) > 0:
                            print(result["scenario"] + " " + result["seed"] + ": Collided")
                            #result["result"] = "COLLISION"
                            thresh_time = data['planning_time'] + rospy.Duration(5)
                            if c.collisions[0] < thresh_time:
                                print("Collision occurred Might have been system error")
                else:
                    print("No 'extra_stuff' in [" + str(result) + "]")
            except Exception as e:
                print(str(e))

            if good_result:
                results_w_planning.append(result)

        self.old_results = self.results
        self.results = results_w_planning

        def my_concat(datas):
            combined = defaultdict(list)

            for d in datas:
                for key,val in d.items():
                    combined[key].append(val)

            merged = {key: np.concatenate(val) for key,val in combined.items()}
            return merged


        cases = filter_results(results=self.results, whitelist={"result":"SUCCEEDED"})
        keys_vals = getValuesForKeys(results=cases, keys=["scenario"])

        defs = get_measurement_dict()
        scales = {"ns":1, "us":1000, "s":1e9, "ms":1e6}
        def get_scale_factor(unit):
            return scales[unit]

        def get_conversion_factor(source_unit, target_unit):
            return get_scale_factor(source_unit)/get_scale_factor(target_unit)

        def reorder_list(input, order=[]):
            output = []
            for k in order:
                if k in input:
                    output.append(k)
                    input.remove(k)
            for k in input:
                output.append(k)
            return output

        order_mappings = defaultdict(list)
        order_mappings['scenario']=['industrial_plant', 'hall_obstacle_course', 'clinic', 'demo_gap']

        #From https://stackoverflow.com/a/6229253
        class smart_dict(dict):
            @staticmethod
            def __missing__(key):
                return key

        display_names = smart_dict()
        display_names.update({'just_checking':'checking', 'total_planning':'planning', 'industrial_plant': 'Industrial', 'clinic': 'Clinic', 'demo_gap':'Barrier', 'hall_obstacle_course': 'Obstacle Course'})

        fig_ind = 0

        des_unit = "ms"
        plt.figure(fig_ind)
        fig_ind+=1

        for key,values in keys_vals.items():
            num_plots = len(values)
            fig, axes = plt.subplots(nrows=1, ncols=num_plots, sharey='row')
            try:
                it = iter(axes)
            except TypeError as te:
                print("Only 1 entry in values, making 'axes' into list")
                axes = [axes]
            #axes = axes if isinstance(axes, list) else [axes]
            plot_ind = 0

            values = reorder_list(input=values, order=order_mappings[key])

            for value in values:
                print("\n" + str(key) + ": " + str(value))
                these_cases = filter_results(results=cases, whitelist={key:value})
                datas = [result['extra_stuff']['statistics'] for result in these_cases if 'extra_stuff' in result and 'statistics' in result['extra_stuff']]
                combined_data = my_concat(datas)

                def print_collision_locations(cases):
                    print("Collision locations:")
                    for case in cases:
                        try:
                            c = case['extra_stuff']['collisions']
                        except KeyError as e:
                            pass
                        else:
                            for p in c.points:
                                print(p)

                print_collision_locations(cases=these_cases)

                def print_stats(data, key):
                    per_key = key+"_period"

                    if key in data and per_key in data:
                        mean_dur = np.mean(data[key])
                        unit = defs[key].unit
                        factor = get_conversion_factor(source_unit=unit, target_unit=des_unit)
                        mean_dur *= factor

                        mean_per = np.mean(data[per_key]) #currently always in ns
                        #factor = get_scale_factor(unit=unit)
                        factor = get_conversion_factor(source_unit="ns", target_unit=des_unit)

                        mean_per *= factor

                        print(str(key) + ": Mean=" + str("{:.2f}". format(mean_dur)) + des_unit)
                        print(str(per_key) + ": Mean=" + str(mean_per) + des_unit)
                        print("Ratio=" + str(mean_per/mean_dur))

                planning_hist_keys = ['just_checking', 'total_planning']
                #display_names = ['checking', 'planning']

                for k in planning_hist_keys:
                    print_stats(combined_data, k)

                def get_scaled_data(k, data=None):
                    data = combined_data if data is None else data
                    vals = data[k]
                    unit = defs[k].unit
                    factor = get_conversion_factor(source_unit=unit, target_unit=des_unit)
                    vals *= factor
                    return vals

                bimodal_vals = [get_scaled_data(k) for k in planning_hist_keys]
                #weights = [np.ones(len(vals)) / len(vals) for vals in bimodal_vals] #This normalizes each set of values independently
                total_num_vals = sum(len(vals) for vals in bimodal_vals)
                weights = [np.ones(len(vals)) / total_num_vals for vals in bimodal_vals] #This normalizes each set of values independently

                ax = axes[plot_ind] #plt.subplot(num_plots,plot_ind,1)
                #axes.append(ax)


                ax.hist(x=bimodal_vals, density=False, weights=weights, label=[display_names[k] for k in planning_hist_keys])
                ax.set_xlabel(xlabel="Time (ms)")
                ax.yaxis.set_major_formatter(PercentFormatter(1))
                ax.set_title(display_names[value])
                fig.set_tight_layout(True)
                fig.set_size_inches(6, 3, forward=True)

                plot_ind += 1
                if plot_ind == len(axes):
                    #handles, labels = ax.get_legend_handles_labels()
                    ax.legend(loc='upper right')

                class AsyncTimingPlotter(object):

                    class TimingDependency(object):
                        def __init__(self, name, dependencies=None, display_name=None):
                            self.name = name
                            self.dependencies = [] if dependencies is None else dependencies if isinstance(dependencies, list) else [dependencies]
                            self.display_name = display_name if display_name is not None else name

                            self.values = None
                            self.mean_duration = None
                            self.start_time =  None
                            self.end_time = None
                            self.parents = []
                            self.children = []

                        def set_data(self, data):
                            self.values = get_scaled_data(self.name, data)
                            #self.values=data[self.name]
                            self.mean_duration = np.mean(self.values)


                        def add_parents(self, dep_objects):
                            for dep in self.dependencies:
                                dep_ob = dep_objects[dep]
                                self.parents.append(dep_ob)
                                dep_ob.children.append(self)

                        def get_end_time(self):
                            if self.end_time is not None: #Have already resolved this value
                                return self.end_time

                            self.start_time = max((d.get_end_time() for d in self.parents), default=0)
                            self.end_time = self.start_time + self.mean_duration
                            return self.end_time

                        def print(self):
                            print(self.name + ": mean=" + str(self.mean_duration) + ", start=" + str(self.start_time) + ", end=" + str(self.end_time))


                    def __init__(self, data):
                        self.deps = {}
                        self.data = data
                        pass


                    def add_key(self, key, dependencies=None, display_name=None):
                        dep = AsyncTimingPlotter.TimingDependency(name=key, dependencies=dependencies, display_name=display_name)
                        self.deps[key]=dep

                    def merge(self, parent_key, child_key, new_key=None, new_display_name=None):
                        parent = self.deps[parent_key]
                        child = self.deps[child_key]
                        if child not in parent.children:
                            raise KeyError("The child key is not a child of the parent key")
                        elif len(parent.children) > 1 or len(child.parents) > 1:
                            raise ValueError("Too many parents or children!")

                        parent.end_time = child.end_time

                        if new_key is not None:
                            del self.deps[parent_key]
                            self.deps[new_key] = parent

                        if new_display_name is not None:
                            parent.display_name = new_display_name

                        del self.deps[child_key]

                        #for grand_child in child.children:



                    def process(self):
                        for dep in self.deps.values():
                            dep.set_data(self.data)
                            dep.add_parents(dep_objects=self.deps)

                        for dep in self.deps.values():
                            dep.get_end_time()

                    def plot(self):
                        for dep in self.deps.values():
                            dep.print()

                        from operator import attrgetter

                        fig, ax = plt.subplots()
                        yheight = 5
                        yinc = 3
                        ymin = 0
                        ylabels = []
                        maxx=0
                        yticks = []
                        wait_times = set()
                        for dep in sorted(self.deps.values(), key=attrgetter('start_time', 'mean_duration')):
                            ax.broken_barh(xranges=[(dep.start_time, dep.mean_duration)], yrange=(ymin, yheight))
                            ylabels.append(dep.display_name)
                            yticks.append(ymin + yheight/2)
                            ymin += (yinc + yheight)
                            maxx = dep.start_time + dep.mean_duration
                            wait_times.add(dep.start_time)

                        ax.set_ylim(0, ymin)
                        ax.invert_yaxis()
                        ax.set_xlim(0, maxx)
                        ax.set_xlabel('Time since depth image(ms)')
                        ax.set_yticks(yticks)
                        ax.set_yticklabels(ylabels)
                        #ax.grid(True)
                        fig.set_tight_layout(True)
                        fig.set_size_inches(6, 3, forward=True)

                        for t in wait_times:
                            plt.axvline(x=t, linestyle='dashed', color='black')



                combined_data["proj_conv"] -= np.mean(combined_data["nd_fm"]) - np.mean(combined_data["fastmarch"])

                #combined_data["ec_update"] = np.array([4.2])

                try:
                    tp = AsyncTimingPlotter(combined_data)
                    tp.add_key("ec_update", display_name="Egocan Update")
                    tp.add_key("ec_2_im", "ec_update")
                    tp.add_key("lib_can_infl", "ec_2_im", display_name="Liberal Egocap Infl")
                    tp.add_key("lib_img_infl", "ec_2_im", display_name="Liberal Egocyl Infl")
                    tp.add_key("cons_img_infl", "ec_2_im", display_name="Cons Egocyl Infl")
                    tp.add_key("cons_can_infl", "ec_2_im", display_name="Cons Egocap Infl")
                    tp.add_key("proj_conv", "ec_2_im", display_name="Egocyl Convolution")

                    tp.add_key("nd_fm", "proj_conv", display_name="Nearest Ranges")

                    tp.add_key("cm_sample", ["ec_2_im", "nd_fm", "proj_conv"], display_name="3DGap")

                    tp.process()
                    tp.merge(parent_key="ec_update", child_key="ec_2_im")
                    tp.plot()
                except KeyError as e:
                    print("Unable to generate timeline due to missing data: " + str(e))


        print("Analyzing fail cases!")
        for failure_type in ["BUMPER_COLLISION", "TIMED_OUT"]:
            cases = filter_results(results=self.results, whitelist={"result": failure_type})
            keys_vals = getValuesForKeys(results=cases, keys=["scenario"])
            for key, values in keys_vals.items():
                values = reorder_list(input=values, order=order_mappings[key])
                for value in values:
                    print("\n" + str(key) + ": " + str(value))
                    these_cases = filter_results(results=cases, whitelist={key: value})

                    print("Final positions: " + str(failure_type))
                    for case in these_cases:
                        try:
                            last_pos = case["extra_stuff"]["last_pos"]
                        except KeyError as e:
                            #print(str(e))
                            pass
                        else:
                            print(str(last_pos) + ": " + str(case['rosbag_file']))


        measurements = {}

        return

        for result in self.results:
            try:
                data = result['extra_stuff']
                c = data["collisions"]
                if len(c.collisions) > 0:
                    print(result["scenario"] + " " + result["seed"] + ": Collided")
            except:
                pass

        for result in self.results:
            try:
                data = result['extra_stuff']
                if "collisions" not in data or len(data["collisions"].collisions==0):
                    s = data["statistics"]

                    self.sa.make_graphs(measurements = s)

                    return
            except:
                pass

        pass


def get_pickled_file_path(filepath):
    import os

    #pickled_path = os.path.join(os.path.dirname(filepath), "cached", os.path.basename(filepath) + ".pickled")
    pickled_path = os.path.join(os.path.dirname(filepath), "cached", os.path.splitext(os.path.basename(filepath))[0] + ".pickled")

    return pickled_path

def preprocess_result(result, force=False):
    import os

    #force = True

    if 'rosbag_file' in result:

        rosbag_file = result['rosbag_file']
        rosbag_file2 = rosbag_file[1:-1]
        if len(rosbag_file2) == 0:
            return

        pickled_path = get_pickled_file_path(filepath=rosbag_file2)

        '''
        try:
            os.remove(pickled_path)
        except OSError as e:
            print(str(e))
        return
        '''

        if force or not os.path.exists(pickled_path):
            print("Processing rosbag [" + rosbag_file2 + "]...")
            in_progress_path = pickled_path + ".active"

            cra = CombinedResultAnalyzer()
            try:
                cra.analyze(bag_file=rosbag_file2, results=[result], log_details=False)
            except ValueError as e:
                print("Problem analyzing [" + str(rosbag_file2) + "]")


            print("Exporting processed data to [" + in_progress_path + "]...")
            cra.export_data(filepath=in_progress_path)

            os.rename(src=in_progress_path, dst=pickled_path)
            print("Renamed processed data to [" + pickled_path + "]")
        else:
            print("Processed data already exists and 'force' is False, skipping [" + rosbag_file2 + "]")
    else:
        print("Skipping result with no rosbag!")


def import_result(result):
    import os

    if 'rosbag_file' in result:

        rosbag_file = result['rosbag_file']
        rosbag_file2 = rosbag_file[1:-1]

        if len(rosbag_file2) == 0:
            return []

        pickled_path = get_pickled_file_path(filepath=rosbag_file2)
        if os.path.exists(pickled_path):
            print("Processing rosbag [" + rosbag_file2 + "]...")

            cra = CombinedResultAnalyzer()

            print("Importing processed data from [" + pickled_path + "]...")
            cra.load_data(filepath=pickled_path)
            results = cra.results
            return results
        else:
            print("Preprocessed data does not exist for rosbag [" + rosbag_file2 + "], did you run preprocessing?")
    else:
        print("Skipping importing preprocessed data for result with no rosbag!")

    return []

def convert_to_measurements(results):
    pass


import itertools

class IndependentBagAnalyzer(CombinedResultAnalyzer):

    def analyze(self, result_file, force=False, single_thread=False):
        analyzer = ResultAnalyzer()
        analyzer.readFiles(filenames=result_file, replacements={'repeat': 'seed'})
        results = analyzer.getCases()

        process_files = True

        cached_result_file = get_pickled_file_path(filepath=result_file+".txt")
        if not force and os.path.exists(cached_result_file):
            try:
                self.load_data(filepath=cached_result_file)
            except EOFError as e:
                print(str(e))
            else:
                process_files = False
                #if len(self.results) == len(results):
                #    process_files = False
                #else:
                #    pass

        if process_files:
            import multiprocessing as mp

            if single_thread:
                itertools.starmap(preprocess_result, zip(results, itertools.repeat(force)))
            else:
                num_cores = min(mp.cpu_count() + 4, len(results))
                #num_cores = 1
                with mp.Pool(num_cores) as pool:
                    pool.starmap(preprocess_result, zip(results, itertools.repeat(force)), 1)

            processed_results = []
            for result in results:
                processed_results += import_result(result=result)

            if len(results) != len(processed_results):
                print("Did not get the right number of processed results!")
            #else:
            self.results = processed_results
            self.export_data(filepath=cached_result_file)

        self.process_data()


def analyze2(reload=False):
    result_file =  '/home/justin/simulation_data/results_2022-03-24 14:21:54.732180'
    bag_file = '/home/justin/simulation_data/iros2022/arxiv/timing_all_2022-03-24-14-22-01.bag' # '/home/justin/simulation_data/iros2022/barrier/demo_gap_all_2022-02-27-18-15-27.bag'
    pickled = '/home/justin/simulation_data/iros2022/arxiv/6timing_all_2022-03-24-14-22-01.pickle'

    a = CombinedResultAnalyzer()
    if reload:
        a.analyze(result_file=result_file, bag_file=bag_file)
        a.export_data(filepath=pickled)
    else:
        a.load_data(filepath=pickled)
        a.process_data()
    pass
    print("Done!")


def analyze3(reload=False):
    bag_file = '/home/justin/.ros/run_2022-04-07-18-50-38.bag'
    pickled = '/home/justin/simulation_data/iros2022/arxiv/run_2022-04-07-18-50-38.pickle'
    result = {'result': 'SUCCEEDED', 'time': '168600000000', 'path_length': '39.451955673496805', 'end_pose': 'NONE',
     'total_rotation': '26.035541316937525', 'scenario':'clinic', 'seed':'0'}
    a = CombinedResultAnalyzer()
    if reload:
        a.analyze(results=[result], bag_file=bag_file)
        a.export_data(filepath=pickled)
    else:
        a.load_data(filepath=pickled)
        a.process_data()
    pass
    print("Done!")


def analyze4(reload=False):
    results_file = '/home/justin/simulation_data/results_2022-04-08 02:20:05.681675'
    pickled = '/home/justin/simulation_data/cached/results_2022-04-08 02:20:05.681675.pickle'
    analyzer = IndependentBagAnalyzer()
    if reload:
        analyzer.analyze(result_file=results_file)
        analyzer.export_data(filepath=pickled)
    else:
        analyzer.load_data(filepath=pickled)
        analyzer.process_data()
    pass
    print("Done!")



def generate_table(results):
    replacements = {'result': {'BUMPER_COLLISION': "BC", 'ABORTED': "AB", "TIMED_OUT": "TO", "SUCCEEDED": "SS"},
                    'global_potential_weight': 'WEIGHT', 'min_obstacle_spacing': 'spacing',
                    }

    a = ResultAnalyzer()
    a.results = results
    a.generateGenericTable(independent=['scenario'], dependent='result', replacements=replacements)


def analyze5(reload=False):
    results_file = '/home/justin/simulation_data/results_2022-04-08 02:20:05.681675' #
    results_file = '/home/justin/simulation_data/results_2022-05-30 01:36:19.386958'
    results_file = '/home/justin/simulation_data/results_2022-05-31 03:02:25.501781'
    results_file = '/home/justin/simulation_data/results_2022-05-31 19:51:50.946400' #Current results
    #results_file = '/home/justin/simulation_data/results_2022-06-08 02:12:33.072331' #with inflated 'detailed' and conservative models
    #results_file = '/home/justin/simulation_data/results_2022-06-09 01:11:46.328783' #same conditions as previous
    #results_file = '/home/justin/simulation_data/results_2022-06-10 00:36:42.597726' #fix goal cost function; change fs_thresh from 100 to 20; shrank base collision model height
    #results_file = '/home/justin/simulation_data/results_2022-06-10 12:36:19.682222' #Fixed goal, but back to original fs_thresh and inflation sizes
    results_file = '/home/justin/simulation_data/results_2022-06-10 20:09:18.058491' #reenabled safety inflation
    results_file = '/home/justin/simulation_data/results_2022-06-11 12:27:17.423103' #slightly increase vertical model size/inflation
    results_file = '/home/justin/simulation_data/results_2022-06-11 20:51:44.099298' #more gentle curves (clinic only)
    #results_file = '/home/justin/simulation_data/results_2022-06-11 23:40:32.445761' #repeat but with all scenarios (failed to run)
    results_file = '/home/justin/simulation_data/results_2022-06-12 07:23:49.580584' #repeat but with all scenarios
    results_file = '/home/justin/simulation_data/results_2022-06-13 00:13:11.833045'

    ##For icra2023
    results_file = '/home/justin/simulation_data/results_2022-09-10 23:59:28.345417' #50 runs in each of 3 worlds; local goal + sampled waypoints + local search
    results_file = '/home/justin/simulation_data/results_2022-09-11 14:51:30.217199' #fewer runs but w/ rosbag recorded
    results_file = '/home/justin/simulation_data/results_2022-09-12 02:50:09.834467' #full set of 50 each, w/o gap-trajectory-commitment
    results_file = '/home/justin/simulation_data/results_2022-09-13 01:10:57.510490' #same, but with resampled waypoint trajectories (5x5 grid of trajectories for each original waypoint)
    #results_file = '/home/justin/simulation_data/results_2022-09-13 19:46:08.883240' #simplified hall, with only collision-free resampled points used
    #results_file = '/home/justin/simulation_data/results_2022-09-14 14:28:02.051699' #Same, but without the extra local search trajectories
    results_file = '/home/justin/simulation_data/results_2022-09-14 18:30:20.279378' #Same, but replacing original waypoints with the center of collision free square of resampled points
    results_file = '/home/justin/simulation_data/results_2022-09-14 23:17:03.986342' #Barrier world experments
    results_file = '/home/justin/simulation_data/results_2022-09-15 01:51:39.237093' #obstacle course + barrier world

    if False:
        file = open(file="/home/justin/simulation_data/rosbags/cached/2022-06-08 02:12:34.081297_663a3da7bb.pickled",
                    mode='rb')
        import pickle
        test = pickle.load(file=file)
        return

    reload = False
    details = True

    if details:
        analyzer = IndependentBagAnalyzer()
        analyzer.analyze(result_file=results_file, force=reload)
        generate_table(analyzer.old_results)
        print("new results:")
        generate_table(analyzer.results)
    else:
        analyzer = ResultAnalyzer()
        analyzer.readFiles(filenames=results_file, replacements={'repeat': 'seed'})
        results = analyzer.getCases()
        generate_table(results)

    print("Done!")






def analyze(file):
    with rosbag.Bag(f=file, mode="r") as bag:
        print("Searching for run start times...")
        rst = RunStartTimesFinder()
        rst.analyze_bag(bag=bag)

        print("Searching for collision times...")
        cst = CollisionFinder()
        cst.analyze_bag(bag=bag)


from nav_scripts.result_analyzer import ResultAnalyzer
if __name__ == "__main__":
    #rospy.init_node('timing_result_analyzer')

    #file = '/home/justin/simulation_data/iros2022/clinic/9/clinic_9_2022-02-26-19-57-54.bag'

    def get_stats(scenario=None, file=None):
        tl = TimingLoader()

        if scenario is not None:
            if scenario == 'demo_gap':
                files = ['/home/justin/simulation_data/iros2022/barrier/demo_gap_all_2022-02-27-18-15-27.bag']
            else:
                files = ['/home/justin/simulation_data/iros2022/' + scenario + '/' + str(run) + ".bag" for run in range(10)]

            for file in files:
                tl.load_from_bag(file=file)
        elif file is not None:
            tl.load_from_bag(file=file)

        tl.make_graphs()


    def find_collisions(scenario):
        if scenario == "industrial_plant":
            file = '/home/justin/simulation_data/iros2022/industrial_plant/industrial_plant_all_2022-02-27-19-26-15.bag'
        elif scenario == "clinic":
            file = '/home/justin/simulation_data/iros2022/clinic/clinic_10_2022-02-28-13-59-31.bag'

        c = RunStartTimesFinder()# CollisionFinder()
        c.analyze_file(file=file)
        print(str(c.collision_starts))

    def get_results(scenario, files=None):
        analyzer = ResultAnalyzer()
        replacements = {'result': {'BUMPER_COLLISION': "BC", 'ABORTED': "AB", "TIMED_OUT": "TO", "SUCCEEDED": "SS"},
                        'global_potential_weight': 'WEIGHT', 'min_obstacle_spacing': 'spacing',
                      }
        if files is None:
            if scenario == 'demo_gap':
                files = ['/home/justin/simulation_data/iros2022/barrier/barrier.results']
            elif scenario == 'industrial_plant':
                files = ['/home/justin/simulation_data/iros2022/industrial_plant/industrial_plant_all.results']
            else:
                runs = range(10)
                #files = ['/home/justin/simulation_data/iros2022/' + scenario + '/' + str(run) + ".results" for run in runs]
                files = ["/home/justin/simulation_data/iros2022/clinic/10_runs.results"]
        analyzer.readFiles(filenames=files, replacements={'repeat':'seed'},blacklist ={'repeat':[28, 42]})
        analyzer.generateGenericTable(independent=['scenario'], dependent='result', replacements=replacements) #

    scenario = "industrial_plant"
    #get_stats(scenario=scenario)
    #get_results(scenario=scenario)

    #get_results('demo_gap')

    #find_collisions(scenario=scenario)

    #get_stats(file="/home/justin/.ros/timing_hallway_2022-03-14-17-09-53.bag")
    #get_stats(file="/home/justin/.ros/timing_clinic_2022-03-20-23-04-15.bag")


    #get_results(scenario=None, files=['/home/justin/simulation_data/results_2022-03-24 14:21:54.732180'])
    #analyze('/home/justin/simulation_data/iros2022/arxiv/timing_all_2022-03-24-14-22-01.bag')

    #analyze3()
    #analyze4()
    analyze5(reload=False)

    plt.show()