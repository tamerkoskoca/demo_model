#!/usr/bin/env python3
"""
BURKUT İHA — Perception Dashboard (pygame)
===========================================
Sunum için tek pencerede:
  Sol  : YOLO kamera görüntüsü (bbox'larla)
  Sağ  : LiDAR top-down harita (direkler + gap)
  Alt  : Gap bilgisi (genişlik / uzaklık / açı)

Çalıştırma:
  source ~/burkut-sim/install/setup.bash
  ros2 run burkut_perception perception_dashboard
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from burkut_msgs.msg import ObstacleArray, GapResult
import pygame
import numpy as np
import math
import threading


# ── Boyutlar ───────────────────────────────────────────────
CAM_W, CAM_H = 640, 480
MAP_W, MAP_H = 580, 480
INFO_H       = 60
WIN_W        = CAM_W + 2 + MAP_W
WIN_H        = CAM_H + INFO_H

MAP_SCALE    = 8.0     # 1 metre = kaç piksel
DIVIDER_X    = CAM_W

# ── Renkler (RGB) ──────────────────────────────────────────
C_BG         = (18,  18,  18)
C_GRID       = (40,  40,  40)
C_POLE       = (220, 60,  60)
C_GAP_LINE   = (50,  220, 80)
C_GAP_LABEL  = (80,  255, 80)
C_PLANE      = (200, 200, 200)
C_HEADING    = (0,   180, 255)
C_WHITE      = (240, 240, 240)
C_GREEN      = (80,  255, 80)
C_YELLOW     = (220, 220, 40)
C_GRAY       = (130, 130, 130)
C_DIVIDER    = (60,  60,  60)
C_RANGE_RING = (50,  50,  50)


def ros_img_to_surface(msg):
    """sensor_msgs/Image → pygame.Surface (cv_bridge olmadan)"""
    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding in ('rgb8',):
            img = data.reshape((msg.height, msg.width, 3))
        elif msg.encoding in ('bgr8',):
            img = data.reshape((msg.height, msg.width, 3))
            img = img[:, :, ::-1].copy()   # BGR → RGB
        else:
            return None
        surf = pygame.surfarray.make_surface(np.transpose(img, (1, 0, 2)))
        return pygame.transform.scale(surf, (CAM_W, CAM_H))
    except Exception:
        return None


class DashboardNode(Node):

    def __init__(self):
        super().__init__('perception_dashboard')

        self.yolo_img  = None
        self.poles     = []
        self.gap       = None
        self._lock     = threading.Lock()

        self.create_subscription(Image,         '/yolo/image',     self._cb_yolo,  10)
        self.create_subscription(ObstacleArray, '/lidar/poles',    self._cb_poles, 10)
        self.create_subscription(GapResult,     '/perception/gap', self._cb_gap,   10)

        self.get_logger().info('Dashboard başlatıldı.')

    def _cb_yolo(self, msg):
        surf = ros_img_to_surface(msg)
        if surf:
            with self._lock:
                self.yolo_img = surf

    def _cb_poles(self, msg):
        with self._lock:
            self.poles = [(obs.x, obs.y) for obs in msg.obstacles]

    def _cb_gap(self, msg):
        with self._lock:
            self.gap = msg


def draw_lidar_panel(screen, poles, gap, font_sm, font_md):
    ox = DIVIDER_X + 2          # panel sol kenar
    cx = ox + MAP_W // 2        # uçak merkezi x
    cy = CAM_H // 2             # uçak merkezi y

    # Grid
    step = int(MAP_SCALE * 5)
    for x in range(ox, ox + MAP_W, step):
        pygame.draw.line(screen, C_GRID, (x, 0), (x, CAM_H))
    for y in range(0, CAM_H, step):
        pygame.draw.line(screen, C_GRID, (ox, y), (ox + MAP_W, y))

    # Menzil çemberleri
    for r_m in [10, 20, 30]:
        r_px = int(r_m * MAP_SCALE)
        pygame.draw.circle(screen, C_RANGE_RING, (cx, cy), r_px, 1)
        lbl = font_sm.render(f'{r_m}m', True, C_GRAY)
        screen.blit(lbl, (cx + r_px + 3, cy - 14))

    # İleri yön oku
    tip_y = cy - int(MAP_SCALE * 6)
    pygame.draw.line(screen, C_HEADING, (cx, cy), (cx, tip_y), 2)
    pygame.draw.polygon(screen, C_HEADING, [
        (cx, tip_y - 8), (cx - 5, tip_y + 2), (cx + 5, tip_y + 2)
    ])

    # Direkler  (ROS: x=ileri, y=sol → ekran: ileri=yukarı, sol=sol)
    for (px, py) in poles:
        sx = cx - int(py * MAP_SCALE)   # y=sol → ekranda sol
        sy = cy - int(px * MAP_SCALE)   # x=ileri → ekranda yukarı
        if ox <= sx < ox + MAP_W and 0 <= sy < CAM_H:
            pygame.draw.circle(screen, C_POLE, (sx, sy), 7)
            pygame.draw.circle(screen, (255, 120, 120), (sx, sy), 7, 2)

    # Gap çizgisi
    if gap and gap.gap_detected:
        ang_rad = math.radians(gap.angle)
        mid_x = cx - int(gap.distance * math.sin(ang_rad) * MAP_SCALE)
        mid_y = cy - int(gap.distance * math.cos(ang_rad) * MAP_SCALE)

        pygame.draw.line(screen, C_GAP_LINE, (cx, cy), (mid_x, mid_y), 2)

        half_w = int(gap.width / 2 * MAP_SCALE)
        p1 = (mid_x - half_w, mid_y)
        p2 = (mid_x + half_w, mid_y)
        pygame.draw.line(screen, C_GAP_LINE, p1, p2, 3)
        pygame.draw.circle(screen, C_GAP_LINE, p1, 5)
        pygame.draw.circle(screen, C_GAP_LINE, p2, 5)

        lbl = font_md.render(f'{gap.width:.1f}m', True, C_GAP_LABEL)
        screen.blit(lbl, (mid_x - 20, mid_y - 22))

    # Uçak noktası
    pygame.draw.circle(screen, C_PLANE, (cx, cy), 6)

    # Panel etiketi
    lbl_bg = pygame.Surface((180, 28))
    lbl_bg.fill((0, 0, 0))
    screen.blit(lbl_bg, (ox, 0))
    lbl = font_sm.render('LiDAR TOP-DOWN', True, C_WHITE)
    screen.blit(lbl, (ox + 8, 6))


def draw_info_band(screen, poles, gap, font_md, font_sm):
    y0 = CAM_H
    pygame.draw.line(screen, C_DIVIDER, (0, y0), (WIN_W, y0))

    if gap and gap.gap_detected:
        sign = '+' if gap.angle >= 0 else ''
        items = [
            ('GAP TESPİT EDİLDİ', C_GREEN),
            (f'Genislik: {gap.width:.2f} m', C_WHITE),
            (f'Uzaklik:  {gap.distance:.1f} m', C_WHITE),
            (f'Aci: {sign}{gap.angle:.1f}°', C_YELLOW),
        ]
    else:
        items = [
            ('GAP YOK', C_GRAY),
            (f'Direk: {len(poles)}', C_GRAY),
            ('', C_GRAY),
            ('', C_GRAY),
        ]

    x = 20
    for text, color in items:
        surf = font_md.render(text, True, color)
        screen.blit(surf, (x, y0 + 16))
        x += 270

    # Sağ köşe
    cnt = font_sm.render(f'Direk: {len(poles)}', True, C_GRAY)
    screen.blit(cnt, (WIN_W - 120, y0 + 20))


def main():
    rclpy.init()
    node = DashboardNode()

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption('BURKUT Perception')
    clock = pygame.time.Clock()

    font_sm = pygame.font.SysFont('monospace', 14)
    font_md = pygame.font.SysFont('monospace', 18, bold=True)

    # ROS2 spin ayrı thread'de
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    waiting_font = pygame.font.SysFont('monospace', 22)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN and event.key == pygame.K_q:
                running = False

        screen.fill(C_BG)

        with node._lock:
            yolo_surf = node.yolo_img
            poles     = list(node.poles)
            gap       = node.gap

        # Sol: kamera
        if yolo_surf:
            screen.blit(yolo_surf, (0, 0))
        else:
            msg = waiting_font.render('YOLO BEKLENIYOR...', True, C_GRAY)
            screen.blit(msg, (CAM_W // 2 - 120, CAM_H // 2))

        # Kamera etiketi
        lbl_bg = pygame.Surface((170, 28))
        lbl_bg.fill((0, 0, 0))
        screen.blit(lbl_bg, (0, 0))
        lbl = font_sm.render('YOLO KAMERA', True, C_WHITE)
        screen.blit(lbl, (8, 6))

        # Dikey ayraç
        pygame.draw.line(screen, C_DIVIDER, (DIVIDER_X, 0), (DIVIDER_X, CAM_H), 2)

        # Sağ: LiDAR
        draw_lidar_panel(screen, poles, gap, font_sm, font_md)

        # Alt bilgi
        draw_info_band(screen, poles, gap, font_md, font_sm)

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
