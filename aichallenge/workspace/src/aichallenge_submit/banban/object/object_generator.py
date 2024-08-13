#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.qos
import rclpy.executors
from std_msgs.msg import Float64MultiArray

class ObjectGeneratorNode(Node):
    def __init__(self):
        super().__init__("object_generator")
        self.pub = self.create_publisher(Float64MultiArray, "/aichallenge/objects", 1)
        self.timer = self.create_timer(1.0, self.publish_objects)  # 1秒ごとに障害物データを送信

    def publish_objects(self):
        msg = Float64MultiArray()
        # サンプルデータ: [x, y, z, 半径] * N 障害物
        msg.data = [
            89644.0, 43152.0, 43.0, 1.5  # 1番目の障害物
        ]
        self.pub.publish(msg)
        #self.get_logger().info(f'Published obstact data: {msg.data}')

class PitStopGeneratorNode(Node):
    def __init__(self):
        super().__init__("pitstop_generator")
        self.pub = self.create_publisher(Float64MultiArray, "/aichallenge/pitstop/area", 1)
        self.timer = self.create_timer(1.0, self.publish_pitstop)  # 1秒ごとに障害物データを送信

    def publish_pitstop(self):
        msg = Float64MultiArray()
        # サンプルデータ: [x, y, z, クォータニオン, スケール] * N 障害物
        msg.data = [
            89626.3671875, 43134.921875, 29.700000762939453, 0.0, 0.0, 0.0, 1.0, 4.0, 2.0
        ]
        self.pub.publish(msg)
        #self.get_logger().info(f'Published pitspot data: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(ObjectGeneratorNode())
    executor.add_node(PitStopGeneratorNode())
    executor.spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# 89644
# 43152
# 43.148
# 道幅 6 m


# marker.pose.position.x = 89626.3671875
#         marker.pose.position.y = 43134.921875
#         marker.pose.position.z = -29.700000762939453
#         marker.pose.orientation.x = 0.0
#         marker.pose.orientation.y = 0.0
#         marker.pose.orientation.z = -0.8788172006607056
#         marker.pose.orientation.w = -0.47715866565704346
#         marker.scale.x = 4.0
#         marker.scale.y = 2.0