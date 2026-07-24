#!/usr/bin/env python
# //==============================================================================
# Standalone utility: drive PSM tool tips to a standardized, centered pose.
#
# Commands each requested PSM's tip to sit `--reach` meters straight out
# along the camera boresight (i.e. dead-center of the camera image), using
# the same tip orientation convention as coordinate_frames.PSM1/2/3.T_tip_cam.
# This does not touch any existing script or file -- it's a one-shot pose
# command you run on top of a running AMBF/surgical_robotics_challenge sim.
# //==============================================================================
import time
from argparse import ArgumentParser

import rclpy
from rclpy.node import Node
from PyKDL import Frame, Vector

from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils import coordinate_frames


def wait_for_ambf_topics(node, timeout=20):
    print("Waiting for AMBF topics to appear and 'CameraFrame' to be registered...")
    SYSTEM_TOPICS = {"/parameter_events", "/rosout"}
    TARGET_PHRASE = "CameraFrame"
    start = time.time()
    while time.time() - start < timeout:
        topics = {name for (name, _) in node.get_topic_names_and_types()}
        non_system = topics - SYSTEM_TOPICS
        camera_found = any(TARGET_PHRASE in t for t in topics)
        if len(non_system) > 0 and camera_found:
            print("Success: AMBF detected and 'CameraFrame' found!")
            return
        time.sleep(0.5)
    raise RuntimeError(f"Timeout: '{TARGET_PHRASE}' was never found.")


def compute_centered_tip_pose(psm, cam, reach_m):
    """
    Tip pose (in the PSM's base frame) that sits `reach_m` meters straight
    out along the camera boresight, i.e. dead-center of the camera image,
    regardless of where this PSM's base happens to be mounted.
    """
    T_cam_to_base = psm.get_T_w_b() * cam.get_T_c_w()
    T_target_cam = Frame(coordinate_frames.PSM1.T_tip_cam.M, Vector(0.0, 0.0, -reach_m))
    return T_cam_to_base * T_target_cam


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='center_psms')
    parser.add_argument('--reach', action='store', dest='reach', type=float, default=0.12,
                         help='Standardized tip distance in front of the camera, in meters (default 0.12)')
    parser.add_argument('--execute-time', action='store', dest='execute_time', type=float, default=1.5,
                         help='Seconds to smoothly interpolate to the target pose (default 1.5)')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Move PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Move PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Move PSM3', default=False)
    parsed_args = parser.parse_args()

    for opt in ('run_psm_one', 'run_psm_two', 'run_psm_three'):
        val = getattr(parsed_args, opt)
        if val in ['True', 'true', '1']:
            setattr(parsed_args, opt, True)
        elif val in ['False', 'false', '0']:
            setattr(parsed_args, opt, False)

    rclpy.init()
    node = Node('center_psms')
    wait_for_ambf_topics(node)

    simulation_manager = SimulationManager(parsed_args.client_name)
    cam = ECM(simulation_manager, 'CameraFrame')
    time.sleep(0.5)

    psms = []
    if parsed_args.run_psm_one:
        psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
        if psm1.is_present():
            psms.append(('psm1', psm1))
    if parsed_args.run_psm_two:
        psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
        if psm2.is_present():
            psms.append(('psm2', psm2))
    if parsed_args.run_psm_three:
        psm3 = PSM(simulation_manager, 'psm3', add_joint_errors=False)
        if psm3.is_present():
            psms.append(('psm3', psm3))

    if len(psms) == 0:
        print('No Valid PSM Arms Specified')
    else:
        for name, psm in psms:
            T_target_b = compute_centered_tip_pose(psm, cam, parsed_args.reach)
            print(f'{name}: commanding tip to {T_target_b.p}, reach={parsed_args.reach} m')
            psm.move_cp(T_target_b, execute_time=parsed_args.execute_time)

        time.sleep(parsed_args.execute_time + 0.5)

        for name, psm in psms:
            print(f'{name}: measured tip pose = {psm.measured_cp().p}')

    node.destroy_node()
    rclpy.shutdown()
