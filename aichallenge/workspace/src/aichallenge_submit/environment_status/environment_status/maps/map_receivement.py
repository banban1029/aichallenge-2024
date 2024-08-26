import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import csv
import os

class MapSubscriber(Node):

    def __init__(self):
        super().__init__('mapping_node')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/map/vector_map_marker',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.map_dir = './workspace/src/aichallenge_submit/environment_status/environment_status/out/'  # 
        if not os.path.exists(self.map_dir):
            os.makedirs(self.map_dir)

    def map_callback(self, msg):
        self.get_logger().info('Received map')
        self.save_map(msg)

    def save_map(self, msg):
        self.get_logger().info('Saving map')
        for marker in msg.markers:
            filename = f'{marker.ns}.csv'
            filepath = os.path.join(self.map_dir, filename)
            with open(filepath, mode='w') as file:
                writer = csv.writer(file)
                writer.writerow(['x', 'y', 'z'])
                for point in marker.points:
                    writer.writerow([point.x, point.y, point.z])
        
def main(args=None):
    rclpy.init(args=args)
    map_subscriber = MapSubscriber()
    rclpy.spin(map_subscriber)
    map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()