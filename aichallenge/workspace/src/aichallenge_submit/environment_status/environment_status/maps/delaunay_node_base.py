#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CenterlineNode(Node):

    def __init__(self):
        super().__init__("centerline_node")

        # 障害物の座標をサブスクライブ
        self.sub = self.create_subscription(Float64MultiArray, "/aichallenge/objects", self.obstacle_callback, 10)
        
        # 障害物のリスト
        self.obstacles = np.array([]).reshape(0, 2)
        
        # マップファイルの読み込み
        self.outer_track_path = './workspace/src/aichallenge_submit/environment_status/environment_status/out//left_lane_bound.csv'
        self.inner_track_path = './workspace/src/aichallenge_submit/environment_status/environment_status/out//right_lane_bound.csv'
        self.center_track_path = './workspace/src/aichallenge_submit/environment_status/environment_status/out//center_lane_line.csv'
        
        self.load_map_files()
        
        # 周期的に中心線を計算・表示するためのタイマー
        self.create_timer(1.0, self.calculate_and_plot_centerline)

    def load_map_files(self):
        column_names = ['x', 'y', 'z']
        self.outer_track_df = pd.read_csv(self.outer_track_path, names=column_names, header=0)
        self.inner_track_df = pd.read_csv(self.inner_track_path, names=column_names, header=0)
        self.center_track_df = pd.read_csv(self.center_track_path, names=column_names, header=0)

    def obstacle_callback(self, msg):
        # コールバックが呼ばれているか確認
        self.get_logger().info("obstacle_callback called")
        
        # 障害物データを更新
        data = np.array(msg.data).reshape(-1, 4)
        self.obstacles = data[:, :2]  # x, y座標のみを使用
        self.get_logger().info(f"Obstacles updated: {self.obstacles}")

    def calculate_and_plot_centerline(self):
        # 外側と内側のポイントを取得
        outer_points = self.outer_track_df[['x', 'y']].to_numpy()
        inner_points = self.inner_track_df[['x', 'y']].to_numpy()

        # 障害物が存在するか確認して、ポイントを結合
        if len(self.obstacles) > 0:
            obstacles_array = np.array(self.obstacles)
            points = np.vstack((outer_points, inner_points, obstacles_array))
        else:
            points = np.vstack((outer_points, inner_points))

        # デロネー三角形分割
        tri = Delaunay(points)

        # 中心線を計算
        path = []
        obstacle_radius = 1.5
        for simplex in tri.simplices:
            vertices = points[simplex]
            for i in range(3):
                p1 = vertices[i]
                p2 = vertices[(i + 1) % 3]
                mid_point = (p1 + p2) / 2

                in_outer = np.any(np.all(outer_points == p1, axis=1)) and np.any(np.all(outer_points == p2, axis=1))
                in_inner = np.any(np.all(inner_points == p1, axis=1)) and np.any(np.all(inner_points == p2, axis=1))

                if in_outer or in_inner:
                    continue

                if len(self.obstacles) > 0:
                    distances = np.linalg.norm(obstacles_array - mid_point, axis=1)

                    if np.min(distances) > obstacle_radius * 1.2:
                        path.append(mid_point)
                else:
                    path.append(mid_point)

        # 中心線をプロット
        self.plot_centerline(path)

    def plot_centerline(self, path):
        path = np.array(path)
        plt.figure(figsize=(10, 6))
        plt.plot(self.outer_track_df['x'], self.outer_track_df['y'], 'black', label='Left Lane Bound')
        plt.plot(self.inner_track_df['x'], self.inner_track_df['y'], 'black', label='Right Lane Bound')
        plt.plot(self.center_track_df['x'], self.center_track_df['y'], 'c--', label='Original Center Lane Line')

        for obs in self.obstacles:
            circle = plt.Circle(obs, 1.5, color='red', alpha=0.5)
            plt.gca().add_artist(circle)

        if len(path) > 0:
            plt.plot(path[:, 0], path[:, 1], 'o', color='coral', label='Calculated Center Line')

        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Track Visualization with Delaunay Triangulation and Center Line')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = CenterlineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()