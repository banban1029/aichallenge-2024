#!/usr/bin/env python3
import rclpy
import rclpy.node
import rclpy.qos
import rclpy.executors
import csv
import os
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from datetime import datetime

class ObjectDetectorNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("object_detector")
        self.sub = self.create_subscription(Float64MultiArray, "/aichallenge/objects", self.callback, 1)
        
        # Prepare CSV file
        output_dir = './workspace/src/aichallenge_submit/environment_status/environment_status/out/'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_file_path = os.path.join(output_dir, f'objects_{timestamp}.csv')

        self.csv_file = open(csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['x', 'y', 'z', 'size'])  # Header of the CSV file

    def callback(self, msg):
        # Log the data length
        self.get_logger().info(f"Received data length: {len(msg.data)}")

        # Log the full data for debugging
        self.get_logger().info(f"Received data: {msg.data}")

        # Log the data
        if len(msg.data) % 4 != 0:
            self.get_logger().warning("Received data length is not a multiple of 4")

        for i in range(0, len(msg.data), 4):
            try:
                x = msg.data[i + 0]
                y = msg.data[i + 1]
                z = msg.data[i + 2]
                size = msg.data[i + 3]
                self.get_logger().info(f"Received object at ({x}, {y}, {z}) with size {size}")

                # Write to CSV file
                self.csv_writer.writerow([x, y, z, size])
                self.csv_file.flush()  
            except IndexError:
                self.get_logger().error(f"Index error processing data at index {i}")

    def close(self):
        # Close CSV file when node is destroyed
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            self.csv_file.close()

class PitStopDetectorNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("pitstop_detector")
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE, durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(Float64MultiArray, "/aichallenge/pitstop/area", self.callback, qos)
        
        # Prepare CSV file
        output_dir = './workspace/src/aichallenge_submit/environment_status/environment_status/out/'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_file_path = os.path.join(output_dir, f'pitstop_{timestamp}.csv')
        self.csv_file = open(csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Position X', 'Position Y', 'Position Z', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W', 'Scale X', 'Scale Y', 'Scale Z'])

    def callback(self, msg):
        # Log the data length
        self.get_logger().info(f"Received pitstop data length: {len(msg.data)}")

        # Log the full data for debugging
        self.get_logger().info(f"Received pitstop data: {msg.data}")

        # Log the data
        if len(msg.data) % 9 != 0:
            self.get_logger().warning("Received pitstop data length is not a multiple of 9")

        for i in range(0, len(msg.data), 9):
            try:
                x = msg.data[i + 0]
                y = msg.data[i + 1]
                z = msg.data[i + 2]
                ox = msg.data[i + 3]
                oy = msg.data[i + 4]
                oz = msg.data[i + 5]
                ow = msg.data[i + 6]
                sx = msg.data[i + 7]
                sy = msg.data[i + 8]
                # Log the data
                self.get_logger().info(f"Received pitstop area at ({x}, {y}, {z}) with orientation ({ox}, {oy}, {oz}, {ow}) and scale ({sx}, {sy}, 0.1)")

                # Write to CSV file
                self.csv_writer.writerow([x, y, z, ox, oy, oz, ow, sx, sy, 0.1])
            except IndexError:
                self.get_logger().error(f"Index error processing pitstop data at index {i}")

    def close(self):
        # Close CSV file when node is destroyed
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            self.csv_file.close()
                                                                                                    


class PitStopConditionNode(rclpy.node.Node):
    def __init__(self):
        super().__init__("pitstop_condition")
        self.sub = self.create_subscription(Int32, "/aichallenge/pitstop/condition", self.callback, 1)
        
        # Prepare CSV file
        output_dir = './workspace/src/aichallenge_submit/environment_status/environment_status/out/'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_file_path = os.path.join(output_dir, f'pitstop_condition_{timestamp}.csv')
        self.csv_file = open(csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Elapsed Time (s)', 'Pit Condition'])  # Header of the CSV file

        # Record the start time
        self.start_time = datetime.now()

    def callback(self, msg):

        # Calculate elapsed time
        current_time = datetime.now()
        elapsed_time = (current_time - self.start_time).total_seconds()
        pit_condition = msg.data

        # Log the data
        self.get_logger().info(f"Received pitstop condition: {pit_condition}")

        # Write to CSV file
        self.csv_writer.writerow([elapsed_time, pit_condition])
        self.csv_file.flush()  

    def close(self):
        # Close CSV file when node is destroyed
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            self.csv_file.close()


class PitstopStatusNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("pitstop_status_node")
        self.sub = self.create_subscription(Float32, "/aichallenge/pitstop/status", self.callback, 1)
        
        # Prepare CSV file
        output_dir = './workspace/src/aichallenge_submit/environment_status/environment_status/out/'
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_file_path = os.path.join(output_dir, f'pitstop_status_{timestamp}.csv')
        self.csv_file = open(csv_file_path, mode='a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Elapsed Time (s)', 'Pit Status'])  # Header of the CSV file

        # Record the start time
        self.start_time = datetime.now()

    def callback(self, msg):
        # Calculate elapsed time
        current_time = datetime.now()
        elapsed_time = (current_time - self.start_time).total_seconds()
        pit_status = msg.data

        # Log the data
        self.get_logger().info(f"Received pitstop status: {pit_status}")

        # Write to CSV file
        self.csv_writer.writerow([elapsed_time, pit_status])
        self.csv_file.flush()  

    def close(self):
        # Close CSV file when node is destroyed
        if hasattr(self, 'csv_file') and self.csv_file is not None:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # CREATE OBJECT DETECTOR NODE
    object_detector_node = ObjectDetectorNode()
    pitstop_detector_node = PitStopDetectorNode()
    pitstop_condition_node = PitStopConditionNode()
    pitstop_status_node = PitstopStatusNode()

    # ADD NODES TO EXECUTOR
    executor.add_node(object_detector_node)
    executor.add_node(pitstop_detector_node)
    executor.add_node(pitstop_condition_node)
    executor.add_node(pitstop_status_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        # Handle ctrl+c
        pass
    finally:
        # Clean up resources
        object_detector_node.close()
        pitstop_detector_node.close()
        pitstop_condition_node.close()
        pitstop_status_node.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
