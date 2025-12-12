# save as ~/rclpy_topic_diag.py and run with the same env you'll use for the client
import time
import rclpy

def list_by_node(node):
    print(">>> get_node_names_and_namespaces()")
    print(node.get_node_names_and_namespaces())

    print(">>> get_publisher_names_and_types_by_node (per-node publishers)")
    pubnames = []
    for [node_name, node_namespace] in node.get_node_names_and_namespaces():
        try:
            pairs = node.get_publisher_names_and_types_by_node(node_name, node_namespace)
            if pairs:
                for p in pairs:
                    print("  node:", node_name, node_namespace, "->", p)
                    pubnames.append(p)
        except Exception as e:
            print("  Exception per-node:", e)

    print(">>> get_topic_names_and_types() (topic-level)")
    print(node.get_topic_names_and_types())

def main():
    rclpy.init()
    node = rclpy.create_node("diag_node")
    try:
        for i in range(10):
            print("=== snapshot", i, "time", time.time(), "===\n")
            list_by_node(node)
            time.sleep(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()