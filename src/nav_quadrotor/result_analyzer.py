#!/usr/bin/env python3

import os
import os.path
import glob
import itertools

import rosbag
import rospy
from nav_scripts.result_analyzer import ResultAnalyzer, getValuesForKeys, filter as filter_results
from pathlib import Path
import multiprocessing as mp

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
        LogMeasurementDefinition(name="ec_update", location="egocylindrical/src/egocylindrical_propagator.cpp:EgoCylindricalPropagator::update:193", prestring="Total time: ", unit="ms"),
        LogMeasurementDefinition(name="ec_2_im", node="/egocylinder/egocylindrical_to_range_image_nodelet", location="egocylindrical/src/range_image_generator.cpp:EgoCylinderRangeImageGenerator::ecPointsCB:106", prestring="Generating egocylindrical range image took ", unit="ms"),

        LogMeasurementDefinition(name="cons_can_infl", node="/egocylinder/conservative_can_image_inflator", location="egocylindrical/src/can_image_inflator_generator.cpp:CanImageInflatorGenerator::imgCB:329", prestring="Inflating egocan lid image by {0.32x0.23} took ", unit="ms"),
        LogMeasurementDefinition(name="cons_img_infl", node="/egocylinder/conservative_range_image_inflator_nodelet", location="egocylindrical/src/range_image_inflator_generator.cpp:RangeImageInflatorGenerator::imgCB:407", prestring="Inflating range image by {0.32x0.23} took ", unit="ms"),

        LogMeasurementDefinition(name="lib_can_infl", node="/egocylinder/liberal_can_image_inflator", location="egocylindrical/src/can_image_inflator_generator.cpp:CanImageInflatorGenerator::imgCB:329", prestring="Inflating egocan lid image by {0.04x0.23} took ", unit="ms"),
        LogMeasurementDefinition(name="lib_img_infl", node="/egocylinder/liberal_range_image_inflator_nodelet", location="egocylindrical/src/range_image_inflator_generator.cpp:RangeImageInflatorGenerator::imgCB:407", prestring="Inflating range image by {0.04x0.23} took ", unit="ms"),

        LogMeasurementDefinition(name="cm_sample", location="endpoint_sampling.py:PointSamplingNode.image_callback:2231", prestring="Waypoint sampling took ", unit="s"),

        LogMeasurementDefinition(name="proj_conv", location="nav_quadrotor/src/freespace_estimator.cpp:update:623", prestring="Total processing time: ", unit="ms"),
        LogMeasurementDefinition(name="fastmarch",
                                 location="nav_quadrotor/src/freespace_estimator.cpp:update:596",
                                 prestring="Fastmarch time: ", unit="ms"),
        LogMeasurementDefinition(name="nd_fm", location="nav_quadrotor/src/freespace_estimator.cpp:update:617", prestring="Nearest Depths processing time: ", unit="ms"),

        LogMeasurementDefinition(name="total_planning", location="trajectory_based_nav/include/trajectory_based_nav/general_nav_impl.h:PlanningData trajectory_based_nav::GeneralNavImpl<T>::Plan:405", prestring="Total: ", unit="ms"),
        LogMeasurementDefinition(name="just_checking",
                                 location="trajectory_based_nav/include/trajectory_based_nav/general_nav_impl.h:PlanningData trajectory_based_nav::GeneralNavImpl<T>::Plan:412",
                                 prestring="STATISTICS: {\"total_planning_time\"", unit="us"),
        LogMeasurementDefinition(name="", location="nav_quadrotor/src/multi_level_trajectory_verifier.cpp:MultiLevelTrajectoryVerifier::collisionCheckPose:133")
    ]
    return meas

from collections import defaultdict
import re
import numpy as np


class DefinitionMatcher:

    def __init__(self, definitions):
        self.func_map = {}
        for d in definitions:
            f = d.function
            try:
                self.func_map[f].append(d)
            except KeyError:
                self.func_map[f] = [d]

    def get_matches(self, msg):
        try:
            func_matches = self.func_map[msg.function]
        except KeyError:
            return []

        file_matches = [d for d in func_matches if msg.file.endswith(d.file)]
        return file_matches


import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

class TimingLoader(object):

    def __init__(self):
        patterns = get_measurement_definitions()
        self.parser = LoggingParser(definitions=patterns)



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



class LoggingParser(object):
    def __init__(self, definitions):
        self.matcher = DefinitionMatcher(definitions=definitions)

        numeric_const_pattern = '[-+]? (?: (?: \d* \. \d+ ) | (?: \d+ \.? ) )(?: [Ee] [+-]? \d+ ) ?'
        self.rx = re.compile(numeric_const_pattern, re.VERBOSE)

    def add_msg(self, msg):
        #Get list of definitions matching msg
        definition_signature_matches = self.matcher.get_matches(msg=msg)
        search_matches = {}

        for mdef in definition_signature_matches:
            if mdef.node is not None and mdef.node !=msg.name:
                continue
            else:
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
        self.parser = LoggingParser(definitions=patterns)

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
                            thresh_time = data['planning_time'] + rospy.Duration(5)
                            if c.collisions[0] < thresh_time:
                                print("Collision occurred near start of run, might have been result of benchmarking system setup error")
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
                    c_pnts = []
                    for case in cases:
                        try:
                            c = case['extra_stuff']['collisions']
                        except KeyError as e:
                            pass
                        else:
                            c_pnts += c.points
                            #for p in c.points:
                            #    print(p)
                    if len(c_pnts)==0:
                        print("No collisions detected in successful runs")

                    else:
                        print("Successful run(s) included collisions, this shouldn't happen. Locations of collisions:")
                        for p in c_pnts:
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

                print("Planning vs checking timing analysis:")
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
                total_num_vals = sum(len(vals) for vals in bimodal_vals)
                weights = np.array([np.ones(len(vals)) / total_num_vals for vals in bimodal_vals], dtype=object) #This normalizes each set of values independently

                ax = axes[plot_ind]

                bimodal_vals = np.array(bimodal_vals, dtype=object)
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
                        ax.set_title(display_names[value])
                        ax.set_yticks(yticks)
                        ax.set_yticklabels(ylabels)
                        fig.set_tight_layout(True)
                        fig.set_size_inches(6, 3, forward=True)

                        for t in wait_times:
                            plt.axvline(x=t, linestyle='dashed', color='black')

                print("Perception pipeline timing analysis:")

                #Not directly recorded so must be inferred
                combined_data["proj_conv"] -= np.mean(combined_data["nd_fm"]) - np.mean(combined_data["fastmarch"])

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


        print("\n\nAnalyzing fail cases:")
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

        return


def get_pickled_file_path(filepath):
    cache_dir = os.path.join(os.path.dirname(filepath), "cached")
    pickled_path = os.path.join(cache_dir, os.path.splitext(os.path.basename(filepath))[0] + ".pickled")
    Path(cache_dir).mkdir(parents=True, exist_ok=True)

    return pickled_path


def preprocess_result(result, force=False):

    if 'rosbag_file' in result:

        rosbag_file = result['rosbag_file']
        rosbag_file2 = rosbag_file[1:-1]
        if len(rosbag_file2) == 0:
            return

        pickled_path = get_pickled_file_path(filepath=rosbag_file2)

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
            if single_thread:
                itertools.starmap(preprocess_result, zip(results, itertools.repeat(force)))
            else:
                num_cores = min(mp.cpu_count() + 4, len(results))
                with mp.Pool(num_cores) as pool:
                    pool.starmap(preprocess_result, zip(results, itertools.repeat(force)), 1)

            processed_results = []
            for result in results:
                processed_results += import_result(result=result)

            if len(results) != len(processed_results):
                print("Some results are missing details! This is fairly normal and includes cases that ended early due required processes crashing")
            #else:
            self.results = processed_results
            self.export_data(filepath=cached_result_file)

        self.process_data()


def generate_table(results):
    replacements = {'result': {'BUMPER_COLLISION': "BC", 'ABORTED': "AB", "TIMED_OUT": "TO", "SUCCEEDED": "SS"},
                    'global_potential_weight': 'WEIGHT', 'min_obstacle_spacing': 'spacing',
                    }

    a = ResultAnalyzer()
    a.results = results
    a.generateGenericTable(independent=['scenario'], dependent='result', replacements=replacements)


##Based on code from https://fabianlee.org/2021/11/11/python-find-the-most-recently-modified-file-matching-a-pattern/
def get_latest_file(data_dir):
    # get list of files that matches pattern
    data_dir = os.path.expanduser(data_dir)
    pattern = os.path.join(data_dir, "results_*")
    files = list(filter(os.path.isfile, glob.glob(pattern)))

    # sort by modified time
    files.sort(key=lambda x: os.path.getmtime(x))

    # get last item in list
    lastfile = files[-1]

    print("Most recent file matching {}: {}".format(pattern, lastfile))
    return lastfile


def analyze(data_dir, results_file=None, reload=False):
    if results_file is None:
        try:
            results_file = get_latest_file(data_dir=data_dir)
        except IndexError:
            print("Error! No matching files found!")
            raise

    analyzer = IndependentBagAnalyzer()
    analyzer.analyze(result_file=results_file, force=reload)
    generate_table(analyzer.results)
    print("Done!")


def main(data_dir="~/simulation_data", results_file=None, reload=False):
    analyze(data_dir=data_dir, results_file = results_file, reload=reload)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()