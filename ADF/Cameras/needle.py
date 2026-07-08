import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
# Change: Import RigidBodyState instead of ObjectState
from ambf_msgs.msg import RigidBodyState, RigidBodyCmd

class NeedleReplayBridge(Node):
    def __init__(self):
        super().__init__('needle_replay_bridge')
        
        # QoS for Jazzy/MCAP bags (usually Best Effort)
        bag_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        ambf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # FIX 1: Changed 'state' to 'State' to match your bag info
        # FIX 2: Changed type to RigidBodyState to match your bag info
        self.state_sub = self.create_subscription(
            RigidBodyState,
            '/ambf/env/phantom/Needle/State',
            self.listener_callback,
            bag_qos)
            
        self.cmd_pub = self.create_publisher(
            RigidBodyCmd, 
            '/ambf/env/phantom/Needle/Command', 
            ambf_qos)
        
        self.get_logger().info("--- Needle Bridge Fixed & Initialized ---")
        self.get_logger().info("Listening on: /ambf/env/phantom/Needle/State")

    def listener_callback(self, msg):
        # This will now trigger because the topic name and type match
        self.get_logger().info(f"Relaying Needle Pose: {msg.pose.position.x:.3f}", once=False)

        cmd = RigidBodyCmd()
        cmd.cartesian_cmd_type = 1 
        cmd.pose = msg.pose
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = NeedleReplayBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()