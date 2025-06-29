#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import heapq

class SimplePlanner(Node):
    def __init__(self):
        super().__init__('simple_planner')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.publisher = self.create_publisher(Path, 'planned_path', 10)
        self.map_info = None
        self.get_logger().info('SimplePlanner node started and listening to /map')

    def map_callback(self, msg):
        self.map_info = msg.info
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        self.get_logger().info(f'Received map of size {width}x{height}')

        start = (10, 10)  # You can adjust based on map5
        goal = (90, 90)

        path = self.a_star(data, start, goal)
        if path:
            self.publish_path(path)
        else:
            self.get_logger().warn('No path found by A*')

    def a_star(self, grid, start, goal):
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
        came_from = {}
        cost_so_far = {start: 0}

        while open_set:
            _, current_cost, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
                neighbor = (current[0]+dx, current[1]+dy)
                if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                    if grid[neighbor] >= 50:
                        continue
                    new_cost = cost_so_far[current] + 1
                    if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                        cost_so_far[neighbor] = new_cost
                        priority = new_cost + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (priority, new_cost, neighbor))
                        came_from[neighbor] = current
        return []

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = p[1] * self.map_info.resolution + self.map_info.origin.position.x
            pose.pose.position.y = p[0] * self.map_info.resolution + self.map_info.origin.position.y
            path_msg.poses.append(pose)
        self.publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimplePlanner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
