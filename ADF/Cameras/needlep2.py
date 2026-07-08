#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ambf_msgs.msg import RigidBodyState
from surgical_robotics_challenge.simulation_manager import SimulationManager
import PyKDL
import time
import sys

class NeedleFinalReplayer(Node):
    def __init__(self):
        super().__init__('needle_replayer')
        
        self.get_logger().info("Connecting to AMBF...")
        self.sim_manager = SimulationManager('needle_replayer_node')
        time.sleep(1.0)
        
        # Using the name we confirmed earlier
        self.needle_handle = self.sim_manager.get_obj_handle('phantom/Needle')
        
        if self.needle_handle is None:
            self.get_logger().error("Could not find 'phantom/Needle'. Trying 'Needle'...")
            self.needle_handle = self.sim_manager.get_obj_handle('Needle')

        if self.needle_handle is None:
            self.get_logger().error("CRITICAL: Needle not found.")
            sys.exit(1)

        # DISABLE PHYSICS for the needle so it doesn't jitter or fall
        # This makes it a 'Kinematic' object
        try:
            self.needle_handle.set_dynamic(False)
            self.get_logger().info("Physics disabled for Needle (Kinematic Mode).")
        except:
            self.get_logger().info("Could not set_dynamic. Proceeding with pose override.")

        self.needle_handle.pub_flag = True 
        self.msg_count = 0

        self.subscription = self.create_subscription(
            RigidBodyState,
            '/ambf/env/phantom/Needle/State',
            self.needle_state_callback,
            10)
        
        self.get_logger().info("Replayer Ready. Play your ROSbag.")

    def needle_state_callback(self, msg):
        self.msg_count += 1
        
        # Extract Pose from Bag
        p = msg.pose.position
        o = msg.pose.orientation
        T_bag = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(o.x, o.y, o.z, o.w),
            PyKDL.Vector(p.x, p.y, p.z)
        )

        # Teleport the needle
        # We use the handle's internal move command if set_pose jitters
        self.needle_handle.set_pose(T_bag)
        
        # Heartbeat every 100 messages
        if self.msg_count % 100 == 0:
            self.get_logger().info(f"Replaying... Frame {self.msg_count}")


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
    replayer = NeedleFinalReplayer()
    try:
        rclpy.spin(replayer)
    except KeyboardInterrupt:
        pass
    finally:
        replayer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


