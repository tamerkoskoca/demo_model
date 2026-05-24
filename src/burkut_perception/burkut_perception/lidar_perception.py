#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from burkut_msgs.msg import Obstacle, ObstacleArray, GapResult
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import math


class LidarPerceptionNode(Node):

    def __init__(self):
        super().__init__('lidar_perception')

        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('cluster_dist', 0.6)
        self.declare_parameter('min_cluster_pts', 2)
        self.declare_parameter('min_gap', 1.5)
        self.declare_parameter('max_gap', 6.0)

        self.max_range    = self.get_parameter('max_range').value
        self.cluster_dist = self.get_parameter('cluster_dist').value
        self.min_pts      = self.get_parameter('min_cluster_pts').value
        self.min_gap      = self.get_parameter('min_gap').value
        self.max_gap      = self.get_parameter('max_gap').value

        self.sub_scan    = self.create_subscription(LaserScan, '/lidar/scan', self.scan_callback, 10)
        self.pub_poles   = self.create_publisher(ObstacleArray, '/lidar/poles', 10)
        self.pub_gap     = self.create_publisher(String, '/perception/gap_result', 10)
        self.pub_gap_msg  = self.create_publisher(GapResult, '/perception/gap', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/lidar/markers', 10)

        self.get_logger().info('LiDAR Perception Node baslatildi.')

    def scan_callback(self, msg: LaserScan):
        frame = msg.header.frame_id if msg.header.frame_id else 'advanced_plane/lidar_link'

        points = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > min(msg.range_max, self.max_range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            points.append((r * math.cos(angle), r * math.sin(angle), r, angle))

        if not points:
            return

        clusters, current = [], [points[0]]
        for pt in points[1:]:
            prev = current[-1]
            if math.sqrt((pt[0]-prev[0])**2 + (pt[1]-prev[1])**2) < self.cluster_dist:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]
        clusters.append(current)

        poles = []
        for cl in clusters:
            if len(cl) < self.min_pts:
                continue
            cx = sum(p[0] for p in cl) / len(cl)
            cy = sum(p[1] for p in cl) / len(cl)
            poles.append((cx, cy, math.sqrt(cx**2+cy**2), math.atan2(cy, cx), len(cl)))
        poles.sort(key=lambda p: p[3])

        gates = []
        for i in range(len(poles) - 1):
            p1, p2 = poles[i], poles[i+1]
            gap_w = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            if self.min_gap <= gap_w <= self.max_gap:
                mid_x = (p1[0]+p2[0]) / 2
                mid_y = (p1[1]+p2[1]) / 2
                gates.append({
                    'width': gap_w,
                    'mid_x': mid_x, 'mid_y': mid_y,
                    'dist': math.sqrt(mid_x**2+mid_y**2),
                    'angle_deg': math.degrees(math.atan2(mid_y, mid_x)),
                    'p1': p1, 'p2': p2,
                })

        best = max(gates, key=lambda g: g['width']) if gates else None
        now = self.get_clock().now().to_msg()

        # ObstacleArray
        obs_msg = ObstacleArray()
        obs_msg.header = Header(); obs_msg.header.stamp = now; obs_msg.header.frame_id = frame
        for cx, cy, cr, ca, npts in poles:
            obs = Obstacle()
            obs.x, obs.y, obs.z = cx, cy, 0.0
            obs.radius, obs.height = 0.15, 33.0
            obs.confidence = min(1.0, npts / 5.0)
            obs.type = 'pole'
            obs_msg.obstacles.append(obs)
        self.pub_poles.publish(obs_msg)

        # MarkerArray
        ma = MarkerArray()
        mid_id = 0

        d = Marker(); d.header.stamp = now; d.header.frame_id = frame
        d.action = Marker.DELETEALL; ma.markers.append(d)

        # Direkler — kısa kırmızı silindir
        for idx, (cx, cy, cr, ca, npts) in enumerate(poles):
            m = Marker()
            m.header.stamp = now; m.header.frame_id = frame
            m.ns = 'poles'; m.id = mid_id; mid_id += 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 3.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.35
            m.scale.y = 0.35
            m.scale.z = 6.0
            m.color.r = 0.9; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.95
            m.lifetime.nanosec = 300_000_000
            ma.markers.append(m)

        # Gap etiketi — büyük, şık, gap olan yerde
        if best:
            t = Marker()
            t.header.stamp = now; t.header.frame_id = frame
            t.ns = 'gap_info'; t.id = mid_id; mid_id += 1
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = best['mid_x']
            t.pose.position.y = best['mid_y']
            t.pose.position.z = 9.0
            t.pose.orientation.w = 1.0
            t.scale.z = 1.8
            t.color.r = 0.1; t.color.g = 1.0; t.color.b = 0.4; t.color.a = 1.0
            angle_sign = '+' if best['angle_deg'] >= 0 else ''
            t.text = (
                f"━━ GAP DETECTED ━━\n"
                f"Genislik  {best['width']:.2f} m\n"
                f"Uzaklik   {best['dist']:.1f} m\n"
                f"Aci       {angle_sign}{best['angle_deg']:.1f}\u00b0"
            )
            t.lifetime.nanosec = 300_000_000
            ma.markers.append(t)

        self.pub_markers.publish(ma)

        # Gap result topic
        if best:
            angle_sign = '+' if best['angle_deg'] >= 0 else ''
            result = (f"Genislik: {best['width']:.2f}m  |  "
                      f"Uzaklik: {best['dist']:.1f}m  |  "
                      f"Aci: {angle_sign}{best['angle_deg']:.1f}\u00b0")
        else:
            result = "GAP YOK"

        gap_msg = String(); gap_msg.data = result
        self.pub_gap.publish(gap_msg)

        gr = GapResult()
        gr.header.stamp = now
        gr.header.frame_id = frame
        gr.gap_detected = best is not None
        gr.width    = float(best['width'])    if best else 0.0
        gr.distance = float(best['dist'])     if best else 0.0
        gr.angle    = float(best['angle_deg']) if best else 0.0
        self.pub_gap_msg.publish(gr)
        self.get_logger().info(result)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
