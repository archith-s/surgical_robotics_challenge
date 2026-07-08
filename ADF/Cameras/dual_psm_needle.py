#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd
from surgical_robotics_challenge.simulation_manager import SimulationManager
import PyKDL
import time

class SmoothZeroGReplayer(Node):
    def __init__(self):
        super().__init__('smooth_replayer')
        
        self.sim_manager = SimulationManager('smooth_kin_node')
        time.sleep(1.0)
        
        # 1. Get Handles
        self.needle = self.sim_manager.get_obj_handle('phantom/Needle')
        self.psm1 = self.sim_manager.get_obj_handle('psm1/toolyawlink')
        self.psm2 = self.sim_manager.get_obj_handle('psm2/toolyawlink')

        # 2. Setup Zero-Gravity State
        for handle in [self.needle, self.psm1, self.psm2]:
            if handle:
                # Use pub_flag=True to ensure the 'teleport' command actually sends
                handle.pub_flag = True 
                try:
                    handle.set_dynamic(False)
                except:
                    # Fix: Pass a list [0, 0, 0] instead of three separate zeros
                    handle.set_force([0.0, 0.0, 0.0])
                    handle.set_torque([0.0, 0.0, 0.0])

        # 3. Subscribers
        self.create_subscription(RigidBodyState, '/ambf/env/phantom/Needle/State', self.needle_cb, 10)
        self.create_subscription(RigidBodyCmd, '/ambf/env/psm1/baselink/Command', self.psm1_cb, 10)
        self.create_subscription(RigidBodyCmd, '/ambf/env/psm2/baselink/Command', self.psm2_cb, 10)

    def msg_to_frame(self, msg_pose):
        p, o = msg_pose.position, msg_pose.orientation
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w), 
                           PyKDL.Vector(p.x, p.y, p.z))

    def needle_cb(self, msg):
        if self.needle:
            self.needle.set_pose(self.msg_to_frame(msg.pose))
            # Optional: continuously zero out velocity to kill physics 'memory'
            # self.needle.set_force([0.0, 0.0, 0.0]) 

    def psm1_cb(self, msg):
        if self.psm1:
            self.psm1.set_pose(self.msg_to_frame(msg.pose))

    def psm2_cb(self, msg):
        if self.psm2:
            self.psm2.set_pose(self.msg_to_frame(msg.pose))

def wait_for_ambf_topics(node, timeout=20):
    print("Waiting for AMBF topics to appear and 'CameraFrame' to be registered...")

    SYSTEM_TOPICS = {"/parameter_events", "/rosout"}
    # The topic name usually follows the pattern: /ambf/env/<object_name>/State
    TARGET_PHRASE = "CameraFrame"

    start = time.time()
    while time.time() - start < timeout:
        topics_and_types = node.get_topic_names_and_types()
        topics = {name for (name, _) in topics_and_types}
        
        # 1. Check if any AMBF topics exist at all
        non_system = topics - SYSTEM_TOPICS
        
        # 2. Specifically look for the CameraFrame in the topic list
        camera_found = any(TARGET_PHRASE in t for t in topics)

        if len(non_system) > 0 and camera_found:
            print(f"Success: AMBF detected and '{TARGET_PHRASE}' found!")
            return
        elif len(non_system) > 0 and not camera_found:
            # Optional: Print a status update every few seconds
            if int(time.time() - start) % 5 == 0:
                print(f"AMBF is running, but '{TARGET_PHRASE}' isn't in the scene yet...")
    
        time.sleep(0.5)

    raise RuntimeError(f"Timeout: AMBF topics appeared, but '{TARGET_PHRASE}' was never found.")

def main(args=None):
    rclpy.init(args=args)
    SYSTEM_TOPICS = {"/parameter_events", "/rosout"}
    node = Node('test_mtm')
    wait_for_ambf_topics(node)
    replayer = SmoothZeroGReplayer()
    try:
        rclpy.spin(replayer)
    except KeyboardInterrupt:
        pass
    finally:
        replayer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()