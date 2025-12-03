#!/usr/bin/env python3
import sys
import os
import glob

# ==============================================================================
# [필수] CARLA Agents 모듈 경로 추가
# ==============================================================================
try:
    sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

current_file_path = os.path.dirname(os.path.abspath(__file__))
agents_path = os.path.abspath(os.path.join(current_file_path, '../../carla'))
sys.path.append(agents_path)
# ==============================================================================

import carla
import rclpy
from rclpy.node import Node
import math
import matplotlib.pyplot as plt

try:
    from agents.navigation.global_route_planner import GlobalRoutePlanner
except ImportError:
    print("[ERROR] 'agents' module not found. Please check your CARLA installation path.")
    exit(1)

class DijkstraPathGenerator(Node):
    def __init__(self):
        super().__init__('dijkstra_path_generator')
        
        try:
            self.host = '127.0.0.1'
            self.port = 2000
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(10.0)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.get_logger().info("Connected to CARLA.")
        except Exception as e:
            self.get_logger().error(f"Connection Failed: {e}")
            return

        spawn_points = self.map.get_spawn_points()
        
        # [설정] 시작점과 도착점
        self.start_tf = spawn_points[0]
        self.goal_tf = spawn_points[100] 

        self.get_logger().info(f"Start: ({self.start_tf.location.x:.2f}, {self.start_tf.location.y:.2f})")
        self.get_logger().info(f"Goal : ({self.goal_tf.location.x:.2f}, {self.goal_tf.location.y:.2f})")

        self.generate_topology_path()

    def generate_topology_path(self):
        self.get_logger().info("Calculating Global Route using CARLA Topology...")
        
        grp = GlobalRoutePlanner(self.map, sampling_resolution=1.0)
        route = grp.trace_route(self.start_tf.location, self.goal_tf.location)
        
        if not route:
            self.get_logger().error("Failed to find path!")
            return

        self.get_logger().info(f"Path Found! Length: {len(route)} waypoints")

        rx, ry = [], []
        for waypoint, road_option in route:
            rx.append(waypoint.transform.location.x)
            ry.append(waypoint.transform.location.y)

        # CSV 저장 (원본 좌표 그대로 저장 - 주행용)
        filename = "global_path.csv"
        file_path = os.path.join(os.getcwd(), filename)

        with open(file_path, "w") as f:
            for x, y in zip(rx, ry):
                f.write(f"{x},{y}\n") 
        
        self.get_logger().info(f"Path saved to {file_path}")

        # [수정됨] 시각화 (X축 좌우 반전 적용)
        if os.environ.get('DISPLAY', '') != '':
            try:
                topology = self.map.get_topology()
                ox, oy = [], []
                for wp1, wp2 in topology:
                    l1, l2 = wp1.transform.location, wp2.transform.location
                    ox.append(l1.x); oy.append(l1.y)
                    ox.append(l2.x); oy.append(l2.y)
                    ox.append(None); oy.append(None)

                plt.figure(figsize=(10,10))
                
                # 1. 맵과 경로 그리기
                plt.plot(ox, oy, "k-", linewidth=0.5, alpha=0.5, label="Roads")
                plt.plot(self.start_tf.location.x, self.start_tf.location.y, "og", markersize=10, label="Start")
                plt.plot(self.goal_tf.location.x, self.goal_tf.location.y, "xb", markersize=10, label="Goal")
                plt.plot(rx, ry, "-r", linewidth=2, label="Legal Path")
                
                # 2. [핵심 수정] X축을 반전시킴 (좌우 반전 효과)
                plt.gca().invert_xaxis() 

                plt.grid(True)
                plt.axis("equal")
                plt.legend()
                plt.title("CARLA Topology Path (X-axis Inverted View)")
                plt.show()
            except Exception as e:
                self.get_logger().warn(f"Plot failed: {e}")

def main():
    rclpy.init()
    node = DijkstraPathGenerator()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()