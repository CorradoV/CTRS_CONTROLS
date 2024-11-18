import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray, Marker


prev_len = 0


def on_marker_array(msg: MarkerArray):
    global prev_len
    new_len = len(msg.markers)
    if new_len != prev_len:
        print(f"length changed: {prev_len} -> {new_len}")
        prev_len = new_len


rclpy.init()
node =Node("debug")
sub = node.create_subscription(MarkerArray, "/cylinders", on_marker_array, 1)
rclpy.spin(node)
