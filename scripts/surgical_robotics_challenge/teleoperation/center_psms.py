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
# Startup mirrors mtm_psm_pair_run.py (wait for ambf topics, build the
# SimulationManager, construct ECM('CameraFrame') and each PSM), but adds
# retries around two AMBF-client races that surfaced as intermittent
# "NAMED OBJECT NOT FOUND" / "outside valid range" crashes -- see
# resolve_simulation_manager() and wait_for_psm_joint_state() below. Those
# races live in SimulationManager/Client.connect() itself (ambf_client.py's
# create_objs_from_rostopics() takes a single, immediate snapshot of
# published topics the instant its node is created, with no settle time for
# that fresh node's DDS discovery to catch up with the rest of the sim), so
# mtm_psm_pair_run.py and control_object.py are exposed to the same races --
# this file just works around them locally instead of patching the shared
# client.
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


def resolve_simulation_manager(base_client_name, required_names, timeout=25, retry_delay=1.0):
    """
    SimulationManager(...) -> Client.connect() creates a brand-new rclpy
    node/DDS participant and, in the same call, immediately snapshots
    get_published_topics() -- with no settle time for that new participant's
    discovery to catch up with everything already running in the sim. That
    snapshot is frozen forever into the object dict (get_obj_handle never
    re-queries the graph), so if it missed an object, no amount of retrying
    get_obj_handle() on the *same* SimulationManager will ever find it.
    Only a brand-new connection attempt (a fresh node) has a chance at a
    fuller snapshot -- so retry the whole connection, not just the lookup.
    """
    attempt = 0
    start = time.time()
    while True:
        attempt += 1
        # Distinct node name per attempt -- reusing a name whose previous
        # attempt's node is still alive in this process causes a rosout
        # publisher-registration collision.
        simulation_manager = SimulationManager(f'{base_client_name}_{attempt}')
        missing = [name for name in required_names if simulation_manager.get_obj_handle(name) is None]
        if not missing:
            return simulation_manager
        print(f"Connection attempt {attempt}: {missing} not yet discovered, reconnecting...")
        if time.time() - start > timeout:
            raise RuntimeError(
                f"Timeout: {missing} never appeared after {attempt} connection attempts. "
                f"This is the AMBF client's topic-discovery race -- try re-running.")
        time.sleep(retry_delay)


def wait_for_psm_joint_state(simulation_manager, name, min_joints=8, timeout=15):
    """
    Poll the PSM's baselink handle until its State message has actually
    arrived (get_joint_names() reports the expected joints) *before*
    constructing PSM(...) -- PSM.__init__ commands joint 6/7 (jaw) as its
    last step, and if no State message has arrived yet the object reports
    zero joints, so the client logs "outside valid range [0 - -1]" and the
    command is silently dropped.
    """
    base_name = f'{name}/baselink'
    start = time.time()
    while time.time() - start < timeout:
        handle = simulation_manager.get_obj_handle(base_name)
        joint_names = handle.get_joint_names() if handle is not None else None
        if joint_names and len(joint_names) >= min_joints:
            return
        time.sleep(0.2)
    raise RuntimeError(
        f"Timeout: '{base_name}' joint state never synced ({min_joints} joints expected). "
        f"This is a transient AMBF discovery race -- try re-running.")


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

    psm_names = []
    if _parse_bool(parsed_args.run_psm_one):
        psm_names.append('psm1')
    if _parse_bool(parsed_args.run_psm_two):
        psm_names.append('psm2')
    if _parse_bool(parsed_args.run_psm_three):
        psm_names.append('psm3')

    rclpy.init()
    # Distinct name from the AMBF client node(s) below (parsed_args.client_name,
    # default 'center_psms') -- reusing the same name causes a rosout
    # publisher-registration collision between the two nodes.
    node = Node('center_psms_topic_check')

    wait_for_ambf_topics(node)

    required_names = ['CameraFrame'] + [f'{name}/baselink' for name in psm_names]
    simulation_manager = resolve_simulation_manager(parsed_args.client_name, required_names)

    cam = ECM(simulation_manager, 'CameraFrame')

    time.sleep(0.5)

    psms = []
    for name in psm_names:
        wait_for_psm_joint_state(simulation_manager, name)
        psm = PSM(simulation_manager, name, add_joint_errors=False)
        if psm.is_present():
            psms.append((name, psm))

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
