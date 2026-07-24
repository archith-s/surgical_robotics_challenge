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
# This does not touch any existing script or file -- it's a one-shot pose
# command you run on top of a running AMBF/surgical_robotics_challenge sim.
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


def wait_for_psm_object_ready(simulation_manager, name, min_joints=8, timeout=15):
    """
    Poll the PSM's baselink object handle until its joint state has actually
    synced (i.e. get_joint_names() reports the expected joints), *before*
    constructing a PSM(...) -- PSM.__init__ commands joint 6/7 (jaw) as its
    last step, and if the object's joint state hasn't arrived yet the client
    reports "outside valid range [0 - -1]" (it thinks the object has zero
    joints) and the command is silently dropped.
    """
    base_name = f'{name}/baselink'
    start = time.time()
    while time.time() - start < timeout:
        handle = simulation_manager.get_obj_handle(base_name)
        if handle is not None:
            joint_names = handle.get_joint_names()
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


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument('-c', action='store', dest='client_name', help='Client Name', default='center_psms')
    parser.add_argument('--settle-time', action='store', dest='settle_time', type=float, default=2.0,
                         help='Seconds to continuously servo toward the target pose (default 2.0)')
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
    # Distinct name from the AMBF client node below (parsed_args.client_name,
    # default 'center_psms') -- reusing the same name causes a rosout
    # publisher-registration collision between the two nodes.
    node = Node('center_psms_topic_check')
    wait_for_ambf_topics(node)
    simulation_manager = SimulationManager(parsed_args.client_name)
    # The AMBF client discovers scene objects on a background thread after
    # connect() -- give that thread a moment to populate the common object
    # namespace before we start looking anything up (same pattern as
    # ObjectControl.__init__ in object_control_gui.py, which sleeps right
    # after connect() for the same reason). Skipping this is what causes the
    # "NAMED OBJECT NOT FOUND" storm where every single lookup fails because
    # the namespace crawl never finished in time.
    time.sleep(1.0)

    # ECM(...) grabs the object handle immediately, but the AMBF client
    # discovers scene objects on a background thread after connect(), so the
    # handle may not be ready yet -- retry rather than racing it. Also, the
    # handle is looked up by name *relative to the common object namespace*,
    # and 'CameraFrame' lives at different depths depending on which ADF/world
    # is loaded (top-level in the standard launch; nested under 'phantom/' in
    # scenes built around camera_generator.py's custom camera rig) -- so try
    # both.
    CAMERA_NAME_CANDIDATES = ['CameraFrame', 'phantom/CameraFrame']
    cam = None
    discover_timeout = 25
    discover_start = time.time()
    while time.time() - discover_start < discover_timeout:
        for candidate in CAMERA_NAME_CANDIDATES:
            if simulation_manager.get_obj_handle(candidate) is not None:
                cam = ECM(simulation_manager, candidate)
                print(f"Resolved camera object as '{candidate}'")
                break
        if cam is not None:
            break
        time.sleep(0.5)
    if cam is None:
        raise RuntimeError(
            f"Timeout: none of {CAMERA_NAME_CANDIDATES} resolved in the AMBF client. "
            f"This is usually a transient AMBF namespace-discovery race -- try re-running.")
    time.sleep(0.5)

    psms = []
    if parsed_args.run_psm_one:
        wait_for_psm_object_ready(simulation_manager, 'psm1')
        psm1 = PSM(simulation_manager, 'psm1', add_joint_errors=False)
        if psm1.is_present():
            psms.append(('psm1', psm1))
    if parsed_args.run_psm_two:
        wait_for_psm_object_ready(simulation_manager, 'psm2')
        psm2 = PSM(simulation_manager, 'psm2', add_joint_errors=False)
        if psm2.is_present():
            psms.append(('psm2', psm2))
    if parsed_args.run_psm_three:
        wait_for_psm_object_ready(simulation_manager, 'psm3')
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
        # mtm_psm_pair_run.py's control loop and object_control_gui.py's
        # ObjectControl.run() loop use -- repeatedly commanding the pose
        # every cycle rather than firing a single interpolated move and
        # walking away from it.
        control_period = 1.0 / CONTROL_RATE
        start_time = time.time()
        while time.time() - start_time < parsed_args.settle_time:
            for name, psm in psms:
                psm.servo_cp(targets[name])
            time.sleep(control_period)

        for name, psm in psms:
            measured = convert_mat_to_frame(psm.measured_cp())
            print(f'{name}: measured tip pose = {measured.p}')

    node.destroy_node()
    rclpy.shutdown()
