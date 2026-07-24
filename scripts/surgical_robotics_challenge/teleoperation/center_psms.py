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
CAMERA_NAME_CANDIDATES = ['CameraFrame', 'phantom/CameraFrame']
JOINT_LABELS = ['j0 yaw', 'j1 pitch', 'j2 insertion', 'j3 tool roll', 'j4 wrist pitch', 'j5 wrist yaw']


class PSMCenterer:
    """
    Connects to a running AMBF/surgical_robotics_challenge sim and drives one
    or more PSM tips to their coag-button "home" pose.
    """

    def __init__(self, client_name='center_psms'):
        # Distinct name from the AMBF client below (client_name, default
        # 'center_psms') -- reusing the same name causes a rosout
        # publisher-registration collision between the two nodes.
        self.node = Node('center_psms_topic_check')
        self._wait_for_ambf_topics()

        self.simulation_manager = SimulationManager(client_name)
        # The AMBF client discovers scene objects on a background thread
        # after connect() -- give that thread a moment to populate the
        # common object namespace before looking anything up (same pattern
        # as ObjectControl.__init__ in object_control_gui.py, which sleeps
        # right after connect() for the same reason). Skipping this is what
        # causes the "NAMED OBJECT NOT FOUND" storm where every lookup fails
        # because the namespace crawl never finished in time.
        time.sleep(1.0)

        self.cam = self._resolve_camera()
        time.sleep(0.5)
        self.psms = {}

    def _wait_for_ambf_topics(self, timeout=20):
        print("Waiting for AMBF topics to appear and 'CameraFrame' to be registered...")
        system_topics = {"/parameter_events", "/rosout"}
        start = time.time()
        while time.time() - start < timeout:
            topics = {name for (name, _) in self.node.get_topic_names_and_types()}
            non_system = topics - system_topics
            if non_system and any('CameraFrame' in t for t in topics):
                print("Success: AMBF detected and 'CameraFrame' found!")
                return
            time.sleep(0.5)
        raise RuntimeError("Timeout: 'CameraFrame' was never found.")

    def _resolve_camera(self):
        # ECM(...) grabs the object handle immediately, but the AMBF client
        # discovers scene objects on a background thread after connect(), so
        # the handle may not be ready yet -- retry rather than racing it.
        # Also, the handle is looked up by name *relative to the common
        # object namespace*, and 'CameraFrame' lives at different depths
        # depending on which ADF/world is loaded (top-level in the standard
        # launch; nested under 'phantom/' in scenes built around
        # camera_generator.py's custom camera rig) -- so try both.
        timeout = 25
        start = time.time()
        while time.time() - start < timeout:
            for candidate in CAMERA_NAME_CANDIDATES:
                if self.simulation_manager.get_obj_handle(candidate) is not None:
                    print(f"Resolved camera object as '{candidate}'")
                    return ECM(self.simulation_manager, candidate)
            time.sleep(0.5)
        raise RuntimeError(
            f"Timeout: none of {CAMERA_NAME_CANDIDATES} resolved in the AMBF client. "
            f"This is usually a transient AMBF namespace-discovery race -- try re-running.")

    def _wait_for_psm_ready(self, name, min_joints=8, timeout=15):
        """
        Poll the PSM's baselink object handle until its joint state has
        actually synced (i.e. get_joint_names() reports the expected joints)
        *before* constructing a PSM(...) -- PSM.__init__ commands joint 6/7
        (jaw) as its last step, and if the object's joint state hasn't
        arrived yet the client reports "outside valid range [0 - -1]" (it
        thinks the object has zero joints) and the command is silently
        dropped.
        """
        base_name = f'{name}/baselink'
        start = time.time()
        while time.time() - start < timeout:
            handle = self.simulation_manager.get_obj_handle(base_name)
            if handle is not None:
                joint_names = handle.get_joint_names()
                if joint_names and len(joint_names) >= min_joints:
                    return
            time.sleep(0.2)
        raise RuntimeError(
            f"Timeout: '{base_name}' joint state never synced ({min_joints} joints expected). "
            f"This is a transient AMBF discovery race -- try re-running.")

    def add_psm(self, name):
        """Connect to the named PSM ('psm1'/'psm2'/'psm3') if it's present in the scene."""
        self._wait_for_psm_ready(name)
        psm = PSM(self.simulation_manager, name, add_joint_errors=False)
        if psm.is_present():
            self.psms[name] = psm
        return psm.is_present()

    def _home_tip_pose(self, name):
        """
        Tip pose (in the PSM's base frame) matching the same coag-button
        home pose used in mtm_psm_pair_run.py, using this PSM's own
        T_tip_cam so its left/right offset from the camera boresight is
        preserved.
        """
        psm = self.psms[name]
        return psm.get_T_w_b() * self.cam.get_T_c_w() * PSM_TIP_CAM_FRAMES[name].T_tip_cam

    def run(self, settle_time=2.0):
        if not self.psms:
            print('No Valid PSM Arms Specified')
            return

        targets = {}
        for name in self.psms:
            targets[name] = self._home_tip_pose(name)
            print(f'{name}: commanding tip to {targets[name].p}')

        # Continuously re-issue servo_cp for settle_time, same mechanism
        # mtm_psm_pair_run.py's control loop and object_control_gui.py's
        # ObjectControl.run() loop use -- repeatedly commanding the pose
        # every cycle rather than firing a single interpolated move and
        # walking away from it.
        control_period = 1.0 / CONTROL_RATE
        start_time = time.time()
        while time.time() - start_time < settle_time:
            for name, psm in self.psms.items():
                psm.servo_cp(targets[name])
            time.sleep(control_period)

        self._print_results()

    def _print_results(self):
        for name, psm in self.psms.items():
            measured = convert_mat_to_frame(psm.measured_cp())
            print(f'{name}: measured tip pose = {measured.p}')
            jp = psm.measured_jp()
            joint_str = ', '.join(f'{label}={val:.4f}' for label, val in zip(JOINT_LABELS, jp))
            print(f'{name}: joint angles (rad/m) = {joint_str}')
            print(f'{name}: raw jp list = {[round(v, 4) for v in jp]}')

    def shutdown(self):
        self.node.destroy_node()


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
    centerer = PSMCenterer(parsed_args.client_name)

    if _parse_bool(parsed_args.run_psm_one):
        centerer.add_psm('psm1')
    if _parse_bool(parsed_args.run_psm_two):
        centerer.add_psm('psm2')
    if _parse_bool(parsed_args.run_psm_three):
        centerer.add_psm('psm3')

    centerer.run(parsed_args.settle_time)
    centerer.shutdown()
    rclpy.shutdown()
