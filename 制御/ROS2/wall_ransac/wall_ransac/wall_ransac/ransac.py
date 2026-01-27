import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ransac_msgs.msg import Ransac, RansacArray
import math
import random
from typing import List, Tuple, Optional

class WallDetectionNode(Node):
    def __init__(self):
        super().__init__('ransac_node')
        
        self.declare_parameter('min_distance_threshold', 0.1)
        self.declare_parameter('max_distance_threshold', 5.0)
        self.declare_parameter('ransac_iterations', 500)
        self.declare_parameter('ransac_threshold', 0.02)
        self.declare_parameter('min_points_for_wall', 30)
        self.declare_parameter('min_wall_length', 0.3)
        self.declare_parameter('max_point_gap', 0.15)
        
        self.min_distance = self.get_parameter('min_distance_threshold').get_parameter_value().double_value
        self.max_distance = self.get_parameter('max_distance_threshold').get_parameter_value().double_value
        self.ransac_iterations = self.get_parameter('ransac_iterations').get_parameter_value().integer_value
        self.ransac_threshold = self.get_parameter('ransac_threshold').get_parameter_value().double_value
        self.min_points_for_wall = self.get_parameter('min_points_for_wall').get_parameter_value().integer_value
        self.min_wall_length = self.get_parameter('min_wall_length').get_parameter_value().double_value
        self.max_point_gap = self.get_parameter('max_point_gap').get_parameter_value().double_value
        
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.wall_publisher = self.create_publisher(
            RansacArray, '/ransac_walls', 10)
        
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/ransac_markers', 10)
        
        self.colors = [
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (1.0, 1.0, 0.0),
            (1.0, 0.0, 1.0),
        ]

    def scan_callback(self, msg: LaserScan):
        try:
            points = self.convert_scan_to_points(msg)
            
            if len(points) < self.min_points_for_wall:
                return
            
            walls = self.detect_walls_ransac(points)
            self.publish_walls(walls, msg.header)
            self.publish_markers(walls, msg.header)
            
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')

    def convert_scan_to_points(self, scan: LaserScan) -> List[Tuple[float, float]]:
        points = []
        
        for i, distance in enumerate(scan.ranges):
            if (math.isnan(distance) or math.isinf(distance) or 
                distance < self.min_distance or distance > self.max_distance):
                continue
            
            angle = scan.angle_min + i * scan.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append((x, y))
        
        return points

    def detect_walls_ransac(self, points: List[Tuple[float, float]]) -> List[dict]:
        walls = []
        remaining_points = points.copy()
        
        wall_id = 0
        while len(remaining_points) >= self.min_points_for_wall and wall_id < 5:
            wall = self.ransac_line_fitting(remaining_points)
            
            if wall is not None:
                walls.append({
                    'id': wall_id,
                    'line_params': wall['line_params'],
                    'inliers': wall['inliers'],
                    'distance_to_origin': wall['distance_to_origin'],
                    'angle': wall['angle'],
                    'length': wall['length'],
                    'endpoints': wall['endpoints']
                })
                
                inlier_set = set(wall['inliers'])
                remaining_points = [p for p in remaining_points if p not in inlier_set]
                wall_id += 1
            else:
                break
        
        return walls

    def ransac_line_fitting(self, points: List[Tuple[float, float]]) -> Optional[dict]:
        if len(points) < 2:
            return None
        
        best_result = None
        best_score = 0
        
        for _ in range(self.ransac_iterations):
            sample_points = random.sample(points, 2)
            p1, p2 = sample_points
            
            if self.point_distance(p1, p2) < 0.1:
                continue
            
            line_params = self.calculate_line_params(p1, p2)
            if line_params is None:
                continue
            
            a, b, c = line_params
            
            inliers = []
            for point in points:
                dist = abs(a * point[0] + b * point[1] + c)
                if dist < self.ransac_threshold:
                    inliers.append(point)
            
            if len(inliers) < self.min_points_for_wall:
                continue
            
            continuous_inliers = self.extract_continuous_segment(inliers, a, b)
            
            if len(continuous_inliers) < self.min_points_for_wall:
                continue
            
            endpoints, length = self.calculate_wall_endpoints(continuous_inliers, a, b)
            
            if length < self.min_wall_length:
                continue
            
            score = len(continuous_inliers) * length
            
            if score > best_score:
                best_score = score
                best_result = {
                    'line_params': line_params,
                    'inliers': continuous_inliers,
                    'distance_to_origin': abs(c),
                    'angle': math.atan2(a, b),
                    'length': length,
                    'endpoints': endpoints
                }
        
        return best_result

    def extract_continuous_segment(self, inliers: List[Tuple[float, float]], a: float, b: float) -> List[Tuple[float, float]]:
        if len(inliers) < 2:
            return inliers
        
        if abs(b) > abs(a):
            sorted_points = sorted(inliers, key=lambda p: p[0])
        else:
            sorted_points = sorted(inliers, key=lambda p: p[1])
        
        segments = []
        current_segment = [sorted_points[0]]
        
        for i in range(1, len(sorted_points)):
            gap = self.point_distance(sorted_points[i], sorted_points[i-1])
            if gap <= self.max_point_gap:
                current_segment.append(sorted_points[i])
            else:
                if len(current_segment) >= self.min_points_for_wall:
                    segments.append(current_segment)
                current_segment = [sorted_points[i]]
        
        if len(current_segment) >= self.min_points_for_wall:
            segments.append(current_segment)
        
        if not segments:
            return []
        
        return max(segments, key=len)

    def calculate_wall_endpoints(self, inliers: List[Tuple[float, float]], a: float, b: float) -> Tuple[Tuple[Tuple[float, float], Tuple[float, float]], float]:
        if abs(b) > abs(a):
            sorted_points = sorted(inliers, key=lambda p: p[0])
        else:
            sorted_points = sorted(inliers, key=lambda p: p[1])
        
        start = sorted_points[0]
        end = sorted_points[-1]
        length = self.point_distance(start, end)
        
        return (start, end), length

    def calculate_line_params(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> Optional[Tuple[float, float, float]]:
        x1, y1 = p1
        x2, y2 = p2
        
        if abs(x1 - x2) < 1e-6 and abs(y1 - y2) < 1e-6:
            return None
        
        a = y2 - y1
        b = x1 - x2
        c = x2 * y1 - x1 * y2
        
        norm = math.sqrt(a**2 + b**2)
        if norm < 1e-6:
            return None
        
        return (a/norm, b/norm, c/norm)

    def point_distance(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def publish_walls(self, walls: List[dict], header):
        if not walls:
            return
        
        msg = RansacArray()
        msg.header = header
        
        for wall in walls:
            wall_msg = Ransac()
            wall_msg.id = wall['id']
            wall_msg.distance = wall['distance_to_origin']
            wall_msg.angle = wall['angle']
            wall_msg.a = wall['line_params'][0]
            wall_msg.b = wall['line_params'][1]
            wall_msg.c = wall['line_params'][2]
            wall_msg.num_points = len(wall['inliers'])
            msg.walls.append(wall_msg)
        
        self.wall_publisher.publish(msg)

    def publish_markers(self, walls: List[dict], header):
        marker_array = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.header = header
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        for wall in walls:
            endpoints = wall['endpoints']
            
            line_marker = Marker()
            line_marker.header = header
            line_marker.ns = "wall_lines"
            line_marker.id = wall['id']
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.05
            
            color = self.colors[wall['id'] % len(self.colors)]
            line_marker.color.r = color[0]
            line_marker.color.g = color[1]
            line_marker.color.b = color[2]
            line_marker.color.a = 1.0
            
            start_point = Point()
            start_point.x = endpoints[0][0]
            start_point.y = endpoints[0][1]
            start_point.z = 0.0
            
            end_point = Point()
            end_point.x = endpoints[1][0]
            end_point.y = endpoints[1][1]
            end_point.z = 0.0
            
            line_marker.points.append(start_point)
            line_marker.points.append(end_point)
            
            marker_array.markers.append(line_marker)
            
            points_marker = Marker()
            points_marker.header = header
            points_marker.ns = "wall_points"
            points_marker.id = wall['id']
            points_marker.type = Marker.POINTS
            points_marker.action = Marker.ADD
            points_marker.scale.x = 0.03
            points_marker.scale.y = 0.03
            
            points_marker.color.r = color[0]
            points_marker.color.g = color[1]
            points_marker.color.b = color[2]
            points_marker.color.a = 0.8
            
            for p in wall['inliers']:
                pt = Point()
                pt.x = p[0]
                pt.y = p[1]
                pt.z = 0.0
                points_marker.points.append(pt)
            
            marker_array.markers.append(points_marker)
            
            text_marker = Marker()
            text_marker.header = header
            text_marker.ns = "wall_text"
            text_marker.id = wall['id']
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            cx = (endpoints[0][0] + endpoints[1][0]) / 2
            cy = (endpoints[0][1] + endpoints[1][1]) / 2
            text_marker.pose.position.x = cx
            text_marker.pose.position.y = cy
            text_marker.pose.position.z = 0.2
            
            text_marker.scale.z = 0.15
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"W{wall['id']}: {wall['distance_to_origin']:.2f}m L:{wall['length']:.2f}m"
            
            marker_array.markers.append(text_marker)
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WallDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
