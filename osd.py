#!/usr/bin/env python3
"""
Вся отрисовка OSD (On-Screen Display) поверх видеокадра. 
Никакой логики управления или расчётов здесь нет — только cv2.
"""
import cv2

class OSDRenderer:
    def __init__(self, width, height):
        self.WIDTH = width
        self.HEIGHT = height
        self.CENTER_X = width // 2
        self.CENTER_Y = height // 2
    
    def draw_crosshair(self, frame):
        cv2.line(frame, (self.CENTER_X, 0), (self.CENTER_X, self.HEIGHT), (100, 100, 100), 1)
        cv2.line(frame, (0, self.CENTER_Y), (self.WIDTH, self.CENTER_Y), (100, 100, 100), 1)
    
    def draw_fps(self, frame, fps):
        text = f"FPS: {fps:.1f}"
        text_w = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0][0]
        pos = (self.WIDTH - text_w - 10, 30)
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, text, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    
    def draw_takeoff(self, frame, remaining_time, thrust_rc):
        cv2.putText(frame, f"TAKEOFF | {remaining_time:.1f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"rate roll/pitch: 0 deg/s | throttle: {thrust_rc}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    def draw_tracking(self, frame, target, err_x_px, err_y_px,
                      rate_roll_deg, rate_pitch_deg, rc_roll, rc_pitch, thrust_rc,
                      gain_scale, target_size, ref_size,
                      phase=None, ttc=None, current_N=None):  # <-- ДОБАВЛЕНО current_N
        if target is None:
            return
        cx, cy, area, (x, y, w, h) = target
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame,
            f"TRACKING | gain x{gain_scale:.2f} | RC {rc_roll}/{rc_pitch}/{thrust_rc}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame,
            f"err_x: {int(err_x_px):+4d}px | rate_roll:  {rate_roll_deg:+6.1f} deg/s",
            (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame,
            f"err_y: {int(err_y_px):+4d}px | rate_pitch: {rate_pitch_deg:+6.1f} deg/s",
            (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame,
            f"target_size: {target_size}px (ref={ref_size})",
            (10, 101), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        status = "CENTERED" if (err_x_px == 0 and err_y_px == 0) else "TRACKING"
        cv2.putText(frame, f"{status}", (10, 128), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        if phase is not None:
            ttc_str = f"{ttc:.2f}s" if ttc is not None else "--"
            # <-- НОВОЕ: Форматируем строку с адаптивным N
            n_str = f" | N={current_N:.2f}" if current_N is not None else ""
            
            cv2.putText(frame,
                f"PN: {phase}{n_str} | TTC {ttc_str}",
                (10, 151), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    
    def draw_aim_point(self, frame, aim_x, aim_y, size=10, color=(0, 255, 255)):
        ax, ay = int(aim_x), int(aim_y)
        cv2.line(frame, (ax - size, ay), (ax + size, ay), color, 2)
        cv2.line(frame, (ax, ay - size), (ax, ay + size), color, 2)
    
    def draw_tracking_lost(self, frame):
        cv2.putText(frame, "TRACKING | TARGET LOST", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    def draw_manual(self, frame, ch1, ch2, ch3, ch4, target):
        cv2.putText(frame, "MANUAL MODE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
        cv2.putText(frame, f"CH: {ch1}/{ch2}/{ch3}/{ch4}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if target is not None:
            x, y, w, h = target[3]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            cv2.putText(frame, "Target visible - toggle ON", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)