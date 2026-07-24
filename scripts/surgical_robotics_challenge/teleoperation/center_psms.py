#!/usr/bin/env python
# //==============================================================================
# Standalone utility: drive PSM tool tips to a standardized, centered pose.
#
# Commands each requested PSM's tip to the same "home" pose that
# mtm_psm_pair_run.py drives it to when the MTM coag button is pressed:
#   T_tip_b = psm.get_T_w_b() * cam.get_T_c_w() * coordinate_frames.PSM{1,2,3}.T_tip_cam
# using that PSM's own T_tip_cam (which is what gives PSM1/PSM2/PSM3 their
# distinct left/right offsets in front of the camera -- collapsing them all
# to one shared transform makes every arm converge on the same point).
#
# Startup here intentionally mirrors mtm_psm_pair_run.py line for line: wait
# for ambf topics (including CameraFrame) to be published, build the
# SimulationManager, construct ECM('CameraFrame') and then each PSM directly
# with no extra polling/retry logic layered on top -- that script is the
# proven-reliable reference, so this one doesn't invent its own startup
# sequence.
# //==============================================================================
import time
from argparse import ArgumentParser

import rclpy
from rclpy.node import Node

from surgical_robotics_challenge.simulation_manager import SimulationManager
from surgical_robotics_challenge.psm_arm import PSM
from surgical_robotics_challenge.ecm_arm import ECM
from surgical_robotics_challenge.utils import coordinate_frames
from surgical_robotics_challenge.utils.utilities import convert_mat_to_frame

PSM_TIP_CAM_FRAMES = {
    'psm1': coordinate_frames.PSM1,
    'psm2': coordinate_frames.PSM2,
    'psm3': coordinate_frames.PSM3,
}

CONTROL_RATE = 100  # Hz, matches the servo loop rate used in mtm_psm_pair_run.py
JOINT_LABELS = ['j0 yaw', 'j1 pitch', 'j2 insertion', 'j3 tool roll', 'j4 wrist pitch', 'j5 wrist yaw']


def wait_for_ambf_topics(node, timeout=20):
    """Identical to mtm_psm_pair_run.py's wait_for_ambf_topics."""
    print("Waiting for AMBF topics to appear and 'CameraFrame' to be registered...")

    SYSTEM_TOPICS = {"/parameter_events", "/rosout"}
    TARGET_PHRASE = "CameraFrame"

    start = time.time()
    while time.time() - start < timeout:
        topics_and_types = node.get_topic_names_and_types()
        topics = {name for (name, _) in topics_and_types}

        non_system = topics - SYSTEM_TOPICS
        camera_found = any(TARGET_PHRASE in t for t in topics)

        if len(non_system) > 0 and camera_found:
            print(f"Success: AMBF detected and '{TARGET_PHRASE}' found!")
            return
        elif len(non_system) > 0 and not camera_found:
            if int(time.time() - start) % 5 == 0:
                print(f"AMBF is running, but '{TARGET_PHRASE}' isn't in the scene yet...")

        time.sleep(0.5)

    raise RuntimeError(f"Timeout: AMBF topics appeared, but '{TARGET_PHRASE}' was never found.")


def compute_home_tip_pose(psm, cam, psm_frame_cls):
    """
    Tip pose (in the PSM's base frame) matching the same coag-button home
    pose used in mtm_psm_pair_run.py, using this PSM's own T_tip_cam so its
    left/right offset from the camera boresight is preserved.
    """
    return psm.get_T_w_b() * cam.get_T_c_w() * psm_frame_cls.T_tip_cam


def _parse_bool(value):
    if value in ['True', 'true', '1', True]:
        return True
    if value in ['False', 'false', '0', False]:
        return False
    return bool(value)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='center_psms')
    parser.add_argument('--settle-time', action='store', dest='settle_time', type=float, default=2.0,
                         help='Seconds to continuously servo toward the target pose (default 2.0)')
    parser.add_argument('--one', action='store', dest='run_psm_one', help='Move PSM1', default=True)
    parser.add_argument('--two', action='store', dest='run_psm_two', help='Move PSM2', default=True)
    parser.add_argument('--three', action='store', dest='run_psm_three', help='Move PSM3', default=False)
    parsed_args = parser.parse_args()

    rclpy.init()
    # Distinct name from the AMBF client node below (parsed_args.client_name,
    # default 'center_psms') -- reusing the same name causes a rosout
    # publisher-registration collision between the two nodes.
    node = Node('center_psms_topic_check')

    wait_for_ambf_topics(node)
    simulation_manager = SimulationManager(parsed_args.client_name)

    cam = ECM(simulation_manager, 'CameraFrame')

    time.sleep(0.5)

    psms = []
    if _parse_bool(parsed_args.run_psm_one):
        psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
        if psm1.is_present():
            psms.append(('psm1', psm1))

    if _parse_bool(parsed_args.run_psm_two):
        psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
        if psm2.is_present():
            psms.append(('psm2', psm2))

    if _parse_bool(parsed_args.run_psm_three):
        psm3 = PSM(simulation_manager, 'psm3', add_joint_errors=False)
        if psm3.is_present():
            psms.append(('psm3', psm3))

    if len(psms) == 0:
        print('No Valid PSM Arms Specified')
    else:
        targets = {}
        for name, psm in psms:
            T_target_b = compute_home_tip_pose(psm, cam, PSM_TIP_CAM_FRAMES[name])
            targets[name] = T_target_b
            print(f'{name}: commanding tip to {T_target_b.p}')

        # Continuously re-issue servo_cp for settle_time, same mechanism
        # mtm_psm_pair_run.py's control loop uses -- repeatedly commanding
        # the pose every cycle rather than firing a single interpolated move
        # and walking away from it.
        control_period = 1.0 / CONTROL_RATE
        start_time = time.time()
        while time.time() - start_time < parsed_args.settle_time:
            for name, psm in psms:
                psm.servo_cp(targets[name])
            time.sleep(control_period)

        for name, psm in psms:
            measured = convert_mat_to_frame(psm.measured_cp())
            print(f'{name}: measured tip pose = {measured.p}')
            jp = psm.measured_jp()
            joint_str = ', '.join(f'{label}={val:.4f}' for label, val in zip(JOINT_LABELS, jp))
            print(f'{name}: joint angles (rad/m) = {joint_str}')
            print(f'{name}: raw jp list = {[round(v, 4) for v in jp]}')

    node.destroy_node()
    rclpy.shutdown()
