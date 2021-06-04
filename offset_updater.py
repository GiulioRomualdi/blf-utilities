
import argparse
import xml.etree.ElementTree as et
from xml.dom import minidom
import re
import math
import numpy as np

import bipedal_locomotion_framework.bindings as blf


def open_polydriver(robot_name, control_board, joints_list):
    param_handler = blf.parameters_handler.StdParametersHandler()
    param_handler.set_parameter_vector_string('joints_list', joints_list)
    param_handler.set_parameter_vector_string('remote_control_boards', [control_board])
    param_handler.set_parameter_string('robot_name', robot_name)
    param_handler.set_parameter_string('local_prefix', 'joint_offset_updater')
    param_handler.set_parameter_bool("check_for_nan", False)
    param_handler.set_parameter_bool("stream_joint_states", True)

    board = blf.robot_interface.construct_remote_control_board_remapper(param_handler)

    bridge = blf.YarpSensorBridge()
    bridge.initialize(param_handler)
    bridge.set_drivers_list([control_board])

    return board, bridge


def get_offsets(xml_root):
    deltas = []
    for node in root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    deltas = [float(x.group()) for x in re.finditer(r'[-+]?[0-9]+(?:.[0-9]+)?', param.text)]

    return deltas


def set_offsets(xml_root, offsets):

    for node in root.findall("group"):
        if node.get('name') == "CALIBRATION":
            for param in node:
                if param.get('name') == "calibrationDelta":
                    param.name = np.array2string(offsets)

    return xml_root

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Automatically update your robot configuration file.')
    parser.add_argument('-i', '--input', type=str, required=True, help='Input xml file')
    parser.add_argument('-o', '--output', type=str, required=True, help='Input xml file')
    parser.add_argument('-b', '--board', type=str, required=True, help='Name of the control board')
    parser.add_argument('-j', '--joints', type=list, required=True, help='Name of the control board')
    parser.add_argument('-r', '--robot', type=str, required=True, help='Name of the robot')

    args = parser.parse_args()

    tree = et.parse(args.input)
    root = tree.getroot()
    offsets = get_offsets(root)

    control_board, sensor_bridge = open_polydriver(args.robot, args.board, args.joints)

    print("Please put the joints in zero configuration.")
    print("Press enter and I will show you the offset.")
    input()

    _, joints_values, _ = sensor_bridge.get_joint_positions()
    print("Offset = " + str(['%.4f' % elem * 180 / math.pi for elem in joints_values]))

    print("Do you want to add this offset to the  already existing one? [y|n]")
    var = input()
    if var != 'y' :
        print("Offset not changed")
        exit()
    else:
        new_offsets = np.array(offsets) + joints_values * 180 / math.pi
        root = set_offsets(root, new_offsets)
        xml_str = minidom.parseString(et.tostring(root, encoding='UTF-8')).toprettyxml(indent='    ')
        with open(args.output, 'w') as f:
            f.write(xml_str)

        print("Offset changed")
