#!/usr/bin/env python


import time
import rospy
from collections import namedtuple
from path_planning_vrep_simulation.msg import XML_PATH, Step, All_XML_PATHS


Point = namedtuple("point", ["x", "y"])
Step_tup = namedtuple("step", ["number",
                           "start",
                           "finish",
                           "duration"])

# robot 1
step_0_1 = Step_tup(0, Point(4, 3), Point(6, 7), 4.472136)
step_1_1 = Step_tup(1, Point(6, 7), Point(6, 7), 0.704833)
step_2_1 = Step_tup(2, Point(6, 7), Point(8, 7), 2.000000)
step_3_1 = Step_tup(3, Point(8, 7), Point(8, 7), 0.590334)
step_4_1 = Step_tup(4, Point(8, 7), Point(11, 3), 5.000000)

# robot 2
step_0_2 = Step_tup(0, Point(4, 6), Point(6, 7), 2.236068)
step_1_2 = Step_tup(1, Point(6, 7), Point(6, 7), 0.295167)
step_2_2 = Step_tup(2, Point(6, 7), Point(8, 7), 2.000000)
step_3_2 = Step_tup(3, Point(8, 7), Point(8, 7), 0.374334)
step_4_2 = Step_tup(4, Point(8, 7), Point(11, 9), 3.605551)

# robot 3
step_0_3 = Step_tup(0, Point(4, 9), Point(4, 7), 2.000000)
step_1_3 = Step_tup(1, Point(4, 7), Point(4, 7), 2.885337)
step_2_3 = Step_tup(2, Point(4, 7), Point(8, 7), 4.000000)
step_3_3 = Step_tup(3, Point(8, 7), Point(8, 7), 0.204833)
step_4_3 = Step_tup(4, Point(8, 7), Point(11, 6), 3.162278)

# robot 4
step_0_4 = Step_tup(0, Point(10, 9), Point(9, 9), 1.000000)
step_1_4 = Step_tup(1, Point(9, 9), Point(9, 9), 7.380349)
step_2_4 = Step_tup(2, Point(9, 9), Point(8, 7), 2.236068)
step_3_4 = Step_tup(3, Point(8, 7), Point(8, 7), 0.704833)
step_4_4 = Step_tup(4, Point(8, 7), Point(6, 7), 2.162278)
step_5_4 = Step_tup(3, Point(6, 7), Point(6, 7), 0.374334)
step_6_4 = Step_tup(4, Point(6, 7), Point(3, 9), 3.605551)

# robot 5
step_0_5 = Step_tup(0, Point(10, 6), Point(10, 9), 6.110142)
step_1_5 = Step_tup(1, Point(10, 6), Point(10, 8), 2.000000)
step_2_5 = Step_tup(2, Point(10, 8), Point(10, 8), 2.002526)
step_3_5 = Step_tup(3, Point(10, 8), Point(8, 7), 2.236068)
step_4_5 = Step_tup(4, Point(8, 7), Point(8, 7), 0.391679)
step_5_5 = Step_tup(4, Point(8, 7), Point(6, 7), 2.000000)
step_6_5 = Step_tup(3, Point(6, 7), Point(6, 7), 0.204833)
step_7_5 = Step_tup(4, Point(6, 7), Point(3, 6), 3.162278)

path_1 = [step_0_1, step_1_1, step_2_1, step_3_1, step_4_1]
path_2 = [step_0_2, step_1_2, step_2_2, step_3_2, step_4_2]
path_3 = [step_0_3, step_1_3, step_2_3, step_3_3, step_4_3]
path_4 = [step_0_4, step_1_4, step_2_4, step_3_4, step_4_4, step_5_4, step_6_4]
path_5 = [step_0_5, step_1_5, step_2_5, step_3_5, step_4_5, step_5_5, step_6_5, step_7_5]


def prepare_path_msg(step_list, robot_id):
    path_msg = XML_PATH()
    path_msg.robot_id = robot_id
    path_msg.path = []
    for step in step_list:
        path_msg.path.append(prepare_step_msg(step))
    return path_msg

def prepare_step_msg(step):
    step_msg = Step()
    step_msg.number = step.number
    step_msg.start = step.start
    step_msg.finish = step.finish
    step_msg.duration = step.duration
    return step_msg

def prepare_final_msg(paths_list):
    final_msg = All_XML_PATHS()
    final_msg.pathes = []
    for path in paths_list:
        final_msg.pathes.append(path)
    return final_msg


msg_1 = prepare_path_msg(path_1, 1)
msg_2 = prepare_path_msg(path_2, 2)
msg_3 = prepare_path_msg(path_3, 3)
msg_4 = prepare_path_msg(path_4, 4)
msg_5 = prepare_path_msg(path_5, 5)

final_msg = prepare_final_msg([msg_1, msg_2, msg_3, msg_4, msg_5])

rospy.init_node("path_from_xml")
pub = rospy.Publisher("path_from_xml", All_XML_PATHS, queue_size=1)

while not rospy.is_shutdown():
    time.sleep(1)
    pub.publish(final_msg)
