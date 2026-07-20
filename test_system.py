#!/usr/bin/env python3
"""
ТЕСТ В РЕАЛЬНОМ ВРЕМЕНИ — показывает все параметры упреждения
во время работы симуляции. Запускайте ОТДЕЛЬНО от main.py.
"""
import cv2
import numpy as np
import subprocess
import time
import math
from config import InterceptConfig
from guidance import InterceptGuidance1
from osd import OSDRenderer
from gimbal_interface import GimbalInterface
from pymavlink import mavutil

# ============================================================================
# КОНФИГ
# ============================================================================
cfg = InterceptConfig()
WIDTH, HEIGHT = cfg.FRAME_W, cfg.FRAME_H
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2

# ============================================================================
# ФУНКЦИИ
# ============================================================================
def connect_mavlink():
    print("🔌 Подключение к ArduPilot SITL...")
    master = mavutil.mavlink_connection(cfg.MAVLINK_CONN)
    master.wait_heartbeat()
    print("✅ Связь установлена")
    return master

def detect_target(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask1, mask2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) > 20:
            x, y, w, h = cv2.boundingRect(c)
            return x + w // 2, y + h // 2, w * h, (x, y, w, h)
    return None

# ============================================================================
# ОСНОВНАЯ ФУНКЦИЯ
# ============================================================================
def main():
    osd = OSDRenderer(WIDTH, HEIGHT)
    guidance = InterceptGuidance1(cfg)
    master = connect_mavlink()
    gimbal = GimbalInterface(master, telemetry_rate_hz=cfg.GIMBAL_TELEMETRY_HZ)

    print("📹 Включение стриминга камеры...")
    subprocess.run(["gz", "topic", "-t", cfg.GAZEBO_CAMERA_TOPIC, "-m", "gz.msgs.Boolean", "-p", "data: 1"], check=False)
    time.sleep(1)

    print(f"🎥 Запуск видеопотока ({WIDTH}x{HEIGHT})...")
    gst_cmd = [
        "gst-launch-1.0", "-q", "udpsrc", f"port={cfg.GST_UDP_PORT}", "!",
        "application/x-rtp, encoding-name=H264, payload=96", "!",
        "rtpjitterbuffer", "!", "rtph264depay", "!", "avdec_h264", "!",
        "videoconvert", "!",
        f"video/x-raw, format=BGR, width={WIDTH}, height={HEIGHT}", "!",
        "fdsink", "fd=1"
    ]
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=10**8)
    frame_size = WIDTH * HEIGHT * 3

    print("=" * 80)
    print("🔬 ТЕСТ В РЕАЛЬНОМ ВРЕМЕНИ: все параметры упреждения")
    print("   Вывод каждые 10 кадров | Нажмите 'q' для выхода")
    print("=" * 80)

    frame_counter = 0

    try:
        while True:
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size:
                continue
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
            frame_counter += 1

            # Обновляем телеметрию гимбала
            gimbal.update()
            gimbal_attitude = gimbal.get_attitude()

            # Детектируем цель
            target = detect_target(frame)

            # Рисуем перекрестие центра
            osd.draw_crosshair(frame)

            if target is not None:
                cx, cy, area, (x, y, w, h) = target
                now = time.time()

                # Вызываем guidance
                g = guidance.update(target, now, gimbal_attitude)

                # Считаем точку упреждения
                aim_x = cx + g["lead_x"]
                aim_y = cy + g["lead_y"]

                # Рисуем рамку цели (зелёная)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # Рисуем центр цели (красная точка)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                # Рисуем крестик упреждения (жёлтый)
                osd.draw_aim_point(frame, aim_x, aim_y)

                # Выводим все параметры каждые 10 кадров
                if frame_counter % 10 == 0:
                    # Получаем внутренние значения для отладки
                    world_az, world_el = guidance._pixel_to_world_angle(cx, cy, gimbal_attitude)
                    los_out = guidance.los.update(world_az, world_el, now)
                    ttc_out = guidance.ttc_est.update(area, now)

                    print(f"\n{'='*80}")
                    print(f"[Frame {frame_counter}]")
                    print(f"🎯 TARGET: cx={cx:4d} cy={cy:4d} area={area:6d} size=({w}x{h})")
                    print(f"   offset from center: ({cx - CENTER_X:+4d}, {cy - CENTER_Y:+4d}) px")
                    print(f"📐 GIMBAL: roll={gimbal_attitude['roll_deg']:+6.2f}° "
                          f"pitch={gimbal_attitude['pitch_deg']:+6.2f}° "
                          f"yaw={gimbal_attitude['yaw_deg']:+6.2f}°")
                    print(f"🌍 WORLD ANGLE: az={world_az:+6.2f}° el={world_el:+6.2f}°")
                    print(f"📊 LOS TRACKER:")
                    print(f"   smoothed: az={los_out['smoothed_cx']:+6.2f}° el={los_out['smoothed_cy']:+6.2f}°")
                    print(f"   rate:     rate_x={los_out['rate_x']:+6.2f}°/s rate_y={los_out['rate_y']:+6.2f}°/s")
                    print(f"⏱️  TTC: ttc={ttc_out['ttc']:.2f}s" if ttc_out['ttc'] else "⏱️  TTC: None")
                    print(f"   growth_rate={ttc_out['growth_rate']:.3f} approaching={ttc_out['approaching']}")
                    print(f"🎯 LEAD:")
                    print(f"   lead_deg: ({guidance._smooth_lead_x:+6.3f}, {guidance._smooth_lead_y:+6.3f})°")
                    print(f"   lead_px:  ({g['lead_x']:+6.2f}, {g['lead_y']:+6.2f}) px")
                    print(f"🎯 AIM POINT: aim_x={aim_x:6.1f} aim_y={aim_y:6.1f}")
                    print(f"   offset from center: ({aim_x - CENTER_X:+6.1f}, {aim_y - CENTER_Y:+6.1f}) px")
                    print(f"   offset from target: ({g['lead_x']:+6.1f}, {g['lead_y']:+6.1f}) px")
                    print(f"📈 STATE: phase={g['phase']} N={g['current_N']:.2f}")
                    print(f"{'='*80}")
            else:
                if frame_counter % 50 == 0:
                    print(f"[Frame {frame_counter}] ЦЕЛЬ НЕ ОБНАРУЖЕНА")

            cv2.imshow("REALTIME TEST — Lead Point Debug", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n🛑 Прервано пользователем")
    except Exception as e:
        print(f"❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        proc.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()