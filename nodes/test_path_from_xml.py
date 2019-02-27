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
step_0_1 = Step_tup(0, Point(4, 9), Point(4, 7), 2.)
step_1_1 = Step_tup(1, Point(4, 7), Point(4, 7), 2.885337)
step_2_1 = Step_tup(2, Point(4, 7), Point(8, 7), 4.)
step_3_1 = Step_tup(3, Point(8, 7), Point(8, 7), 0.204833)
step_4_1 = Step_tup(4, Point(8, 7), Point(11, 6), 3.162278)

# robot 2
step_0_2 = Step_tup(0, Point(4, 6), Point(6, 7), 2.236068)
step_1_2 = Step_tup(1, Point(6, 7), Point(6, 7), 0.295167)
step_2_2 = Step_tup(2, Point(6, 7), Point(8, 7), 2.000000)
step_3_2 = Step_tup(3, Point(8, 7), Point(8, 7), 0.374334)
step_4_2 = Step_tup(4, Point(8, 7), Point(11, 9), 3.605551)

path_1 = [step_0_1, step_1_1, step_2_1, step_3_1, step_4_1]
path_2 = [step_0_2, step_1_2, step_2_2, step_3_2, step_4_2]


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

final_msg = prepare_final_msg([msg_1, msg_2])

rospy.init_node("path_from_xml")
pub = rospy.Publisher("path_from_xml", All_XML_PATHS, queue_size=1)

while not rospy.is_shutdown():
    time.sleep(1)
    pub.publish(final_msg)
