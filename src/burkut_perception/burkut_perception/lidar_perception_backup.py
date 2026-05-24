#!/usr/bin/env python3
"""
LiDAR Perception Node
/lidar/scan → direk tespiti → /lidar/poles (ObstacleArray)
                             → /perception/gap_result (String)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String
from burkut_msgs.msg import Obstacle, ObstacleArray
import math


class LidarPerceptionNode(Node):

    def __init__(self):
        super().__init__('lidar_perception')

        # --- Parametreler ---
        self.declare_parameter('max_range', 30.0)        # metre — bu kadar uzaktakiler yok sayılır
        self.declare_parameter('cluster_dist', 0.6)      # metre — cluster ayırma eşiği
        self.declare_parameter('min_cluster_pts', 2)     # minimum nokta sayısı (gürültü filtresi)
        self.declare_parameter('min_gap', 2.5)           # metre — geçilebilir minimum gap

        self.max_range   = self.get_parameter('max_range').value
        self.cluster_dist= self.get_parameter('cluster_dist').value
        self.min_pts     = self.get_parameter('min_cluster_pts').value
        self.min_gap     = self.get_parameter('min_gap').value

        # --- Publisher / Subscriber ---
        self.sub_scan = self.create_subscription(
            LaserScan, '/lidar/scan', self.scan_callback, 10)

        self.pub_poles = self.create_publisher(
            ObstacleArray, '/lidar/poles', 10)

        self.pub_gap = self.create_publisher(
            String, '/perception/gap_result', 10)

        self.get_logger().info('LiDAR Perception Node başlatıldı.')

    # ------------------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # 1) Geçerli noktaları Kartezyen'e çevir
        points = []
        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > min(msg.range_max, self.max_range):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, r, angle))

        if not points:
            return

        # 2) Basit mesafe bazlı clustering (1D angular sıralı)
        # points zaten açısal sıralı geliyor
        clusters = []
        current = [points[0]]

        for pt in points[1:]:
            prev = current[-1]
            dist = math.sqrt((pt[0]-prev[0])**2 + (pt[1]-prev[1])**2)
            if dist < self.cluster_dist:
                current.append(pt)
            else:
                clusters.append(current)
                current = [pt]
        clusters.append(current)

        # 3) Küçük cluster'ları filtrele, centroid hesapla
        poles = []  # (cx, cy, range, angle, npts)
        for cl in clusters:
            if len(cl) < self.min_pts:
                continue
            cx = sum(p[0] for p in cl) / len(cl)
            cy = sum(p[1] for p in cl) / len(cl)
            cr = math.sqrt(cx**2 + cy**2)
            ca = math.atan2(cy, cx)
            poles.append((cx, cy, cr, ca, len(cl)))

        # 4) Açıya göre sırala (sol → sağ)
        poles.sort(key=lambda p: p[3])

        # 5) Gap analizi — komşu direkler arası mesafe
        gaps = []
        for i in range(len(poles) - 1):
            p1 = poles[i]
            p2 = poles[i + 1]
            # İki direk arasındaki Kartezyen mesafe
            gap_w = math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
            # Gap merkezi
            mid_x = (p1[0] + p2[0]) / 2
            mid_y = (p1[1] + p2[1]) / 2
            mid_r = math.sqrt(mid_x**2 + mid_y**2)
            mid_a = math.atan2(mid_y, mid_x)
            gaps.append({
                'width': gap_w,
                'mid_x': mid_x,
                'mid_y': mid_y,
                'mid_range': mid_r,
                'mid_angle_deg': math.degrees(mid_a),
                'pole_left': i,
                'pole_right': i + 1,
            })

        # 6) ObstacleArray yayınla
        obs_msg = ObstacleArray()
        obs_msg.header = Header()
        obs_msg.header.stamp = self.get_clock().now().to_msg()
        obs_msg.header.frame_id = 'advanced_plane/lidar_link'

        for idx, (cx, cy, cr, ca, npts) in enumerate(poles):
            obs = Obstacle()
            obs.x = cx
            obs.y = cy
            obs.z = 0.0
            obs.radius = 0.15
            obs.height = 33.0
            obs.confidence = min(1.0, npts / 5.0)
            obs.type = 'pole'
            obs_msg.obstacles.append(obs)

        self.pub_poles.publish(obs_msg)

        # 7) Gap result yayınla
        passable = [g for g in gaps if g['width'] >= self.min_gap]

        lines = [f'=== LiDAR Gap Detection ===']
        lines.append(f'Tespit edilen direk: {len(poles)}')
        lines.append(f'Toplam gap: {len(gaps)}, Geçilebilir: {len(passable)}')

        if poles:
            lines.append('\n--- Direkler (lidar frame) ---')
            for i, (cx, cy, cr, ca, npts) in enumerate(poles):
                lines.append(
                    f'  Direk {i}: mesafe={cr:.1f}m  aci={math.degrees(ca):+.1f}°  '
                    f'pos=({cx:.1f},{cy:.1f})'
                )

        if gaps:
            lines.append('\n--- Gapler ---')
            for g in gaps:
                flag = '✓ GEÇİLEBİLİR' if g['width'] >= self.min_gap else '✗ dar'
                lines.append(
                    f'  Gap {g["pole_left"]}-{g["pole_right"]}: '
                    f'genislik={g["width"]:.2f}m  '
                    f'merkez: mesafe={g["mid_range"]:.1f}m  aci={g["mid_angle_deg"]:+.1f}°  '
                    f'{flag}'
                )

        if passable:
            best = max(passable, key=lambda g: g['width'])
            lines.append(
                f'\n>>> EN IYI GAP: {best["width"]:.2f}m  '
                f'aci={best["mid_angle_deg"]:+.1f}°  '
                f'mesafe={best["mid_range"]:.1f}m'
            )
        else:
            lines.append('\n>>> Geçilebilir gap YOK')

        gap_msg = String()
        gap_msg.data = '\n'.join(lines)
        self.pub_gap.publish(gap_msg)

        # Log (kısa)
        self.get_logger().info(
            f'{len(poles)} direk | {len(passable)}/{len(gaps)} gap geçilebilir'
        )


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
