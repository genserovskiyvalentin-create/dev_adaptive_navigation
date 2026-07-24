#!/usr/bin/env python3
"""
COMPANION SCRIPT - PD-ТРЕКЕР ACRO + PN (АДАПТИВНЫЙ, УГЛОВОЙ, С ГИМБАЛОМ)
DEBUG VERSION - с подробным выводом всех промежуточных значений
"""
import cv2
import numpy as np
import subprocess
import time
import pygame
import math
from pymavlink import mavutil
from config import InterceptConfig
from guidance import InterceptGuidance1
from osd import OSDRenderer
from gimbal_interface import GimbalInterface

# ============================================================================
# КОНФИГ
# ============================================================================
cfg = InterceptConfig()
WIDTH, HEIGHT = cfg.FRAME_W, cfg.FRAME_H
CENTER_X = WIDTH // 2
CENTER_Y = HEIGHT // 2
MAX_RATE_RAD_S = math.radians(cfg.MAX_RATE_DEG_S)
MAX_RATE_CMD_RATE = math.radians(cfg.MAX_RATE_CMD_RATE_DEG_S2)

# DEBUG: включаем подробный вывод
DEBUG_MODE = True
DEBUG_EVERY_N_FRAMES = 30  # вывод каждые 30 кадров

# ============================================================================
# ФУНКЦИИ
# ============================================================================
def connect_mavlink():
    print("🔌 Подключение к ArduPilot SITL...")
    master = mavutil.mavlink_connection(cfg.MAVLINK_CONN)
    master.wait_heartbeat()
    print("✅ Связь установлена")
    return master

def init_joystick():
    pygame.init()
    pygame.joystick.init()
    count = pygame.joystick.get_count()
    if count == 0:
        print("❌ Джойстик не найден!")
        return None
    for i in range(count):
        joy = pygame.joystick.Joystick(i)
        joy.init()
        if 'uinput' in joy.get_name().lower() or 'virtual' in joy.get_name().lower():
            return joy
    joy = pygame.joystick.Joystick(0)
    joy.init()
    return joy

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

def read_joystick(joy):
    pygame.event.pump()
    def apply_dz(v): return 0.0 if abs(v) < cfg.JOY_DEADZONE else v
    roll = apply_dz(joy.get_axis(3))
    pitch = apply_dz(joy.get_axis(2))
    throttle = apply_dz(joy.get_axis(1))
    yaw = apply_dz(joy.get_axis(0))
    if cfg.INVERT_PITCH: pitch = -pitch
    if cfg.INVERT_YAW: yaw = -yaw
    if cfg.INVERT_ROLL: roll = -roll
    def axis_to_rc(v): return max(1000, min(2000, int(v * 500 + 1500)))
    ch1, ch2, ch3, ch4 = axis_to_rc(roll), axis_to_rc(pitch), axis_to_rc(throttle), axis_to_rc(yaw)
    switch_on, ch5 = False, 1500
    if joy.get_numaxes() > 4:
        switch_raw = joy.get_axis(4)
        ch5 = axis_to_rc(switch_raw)
        switch_on = switch_raw > cfg.SWITCH_THRESHOLD
    return ch1, ch2, ch3, ch4, ch5, switch_on

def send_rc_override(master, ch1, ch2, ch3, ch4, ch5):
    master.mav.rc_channels_override_send(master.target_system, master.target_component, ch1, ch2, ch3, ch4, ch5, 0, 0, 0)

def rate_to_rc(rate_rad_s):
    normalized = max(-1.0, min(1.0, math.degrees(rate_rad_s) / cfg.ACRO_RP_RATE_DEG_S))
    return max(1000, min(2000, int(1500 + normalized * 500)))

def get_drone_attitude(master):
    last_roll, last_pitch = None, None
    while True:
        msg = master.recv_match(type='ATTITUDE', blocking=False)
        if msg is None: break
        last_roll, last_pitch = msg.roll, msg.pitch
    return (last_roll or 0.0), (last_pitch or 0.0)

# ============================================================================
# ОСНОВНАЯ ФУНКЦИЯ
# ============================================================================
def main():
    osd = OSDRenderer(WIDTH, HEIGHT)
    guidance = InterceptGuidance1(cfg)
    master = connect_mavlink()
    gimbal = GimbalInterface(master, telemetry_rate_hz=cfg.GIMBAL_TELEMETRY_HZ)
    joy = init_joystick()
    if joy is None: return

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

    last_switch_on, auto_active, target_lost_counter = False, False, 0
    takeoff_active, takeoff_start_time = False, 0.0
    prev_err_x, prev_err_y, prev_rate_roll, prev_rate_pitch, prev_time = 0.0, 0.0, 0.0, 0.0, time.time()
    
    # DEBUG: счётчик кадров
    frame_counter = 0

    print("=" * 60)
    print("🤖 PD-ТРЕКЕР ACRO + PN (АДАПТИВНЫЙ, УГЛОВОЙ, С ГИМБАЛОМ)")
    print(f"🔧 DEBUG MODE: вывод каждые {DEBUG_EVERY_N_FRAMES} кадров")
    print("=" * 60)

    try:
        while True:
            raw_frame = proc.stdout.read(frame_size)
            if len(raw_frame) != frame_size: continue
            frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape(HEIGHT, WIDTH, 3).copy()
            
            frame_counter += 1
            is_debug_frame = DEBUG_MODE and (frame_counter % DEBUG_EVERY_N_FRAMES == 0)

            # Обновляем телеметрию гимбала
            gimbal.update()
            gimbal_attitude = gimbal.get_attitude()


            # === НЕПРЕРЫВНЫЙ ВЫВОД УГЛОВ ГИМБАЛА НА ЭКРАН ===
            roll_g = gimbal_attitude.get("roll_deg", 0.0)
            pitch_g = gimbal_attitude.get("pitch_deg", 0.0)
                        
            # Формируем строку для вывода
            info_text = f"GIMBAL: R:{roll_g:+5.1f}° P:{pitch_g:+5.1f}°"
            
            # Рисуем текст в левом верхнем углу (координаты x=10, y=20)
            # Цвет (255, 255, 0) - желтый, толщина 1
            cv2.putText(frame, info_text, (10, 200), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)



            

            ch1, ch2, ch3, ch4, ch5, switch_on = read_joystick(joy)
            target = detect_target(frame)
            osd.draw_crosshair(frame)

            # --- ПЕРЕКЛЮЧЕНИЕ РЕЖИМОВ ---
            if switch_on and not last_switch_on:
                current_roll, current_pitch = get_drone_attitude(master)
                safe_limit_rad = math.radians(cfg.SAFE_ANGLE_LIMIT_DEG)
                print(f"\n🔘 ТУМБЛЕР ВКЛ → TAKEOFF (roll: {math.degrees(np.clip(current_roll, -safe_limit_rad, safe_limit_rad)):+.1f}°)")
                takeoff_active, takeoff_start_time, auto_active, target_lost_counter = True, time.time(), False, 0

            if not switch_on and (auto_active or takeoff_active):
                print("\n🔘 ТУМБЛЕР ВЫКЛ → MANUAL")
                auto_active, takeoff_active, target_lost_counter = False, False, 0

            if takeoff_active and (time.time() - takeoff_start_time) >= cfg.TAKEOFF_TIME_SEC:
                print("\n✅ TAKEOFF завершён → TRACKING")
                takeoff_active, auto_active, target_lost_counter = False, True, 0
                prev_err_x, prev_err_y, prev_rate_roll, prev_rate_pitch, prev_time = 0.0, 0.0, 0.0, 0.0, time.time()

            if auto_active and target is None:
                target_lost_counter += 1
                if target_lost_counter >= cfg.TARGET_LOST_FRAMES:
                    print("\n🎯 ЦЕЛЬ ПОТЕРЯНА → MANUAL")
                    auto_active, target_lost_counter = False, 0
                    guidance.reset()

            elif auto_active and target is not None:
                target_lost_counter = 0

            last_switch_on = switch_on

            # --- ФАЗА TAKEOFF ---
            if takeoff_active:
                send_rc_override(master, 1500, 1500, cfg.THRUST_RC, 1500, 1500)
                osd.draw_takeoff(frame, max(0, cfg.TAKEOFF_TIME_SEC - (time.time() - takeoff_start_time)), cfg.THRUST_RC)

            # --- ФАЗА TRACKING ---
            elif auto_active:
                if target is not None:
                    cx, cy, area, (x, y, w, h) = target
                    
                    # DEBUG: вывод показаний гимбала
                    if is_debug_frame:
                        print(f"\n{'='*60}")
                        print(f"[DEBUG Frame {frame_counter}]")
                        print(f"📐 GIMBAL: pitch={gimbal_attitude['pitch_deg']:+.2f}° | "
                              f"yaw={gimbal_attitude['yaw_deg']:+.2f}° | "
                              f"roll={gimbal_attitude['roll_deg']:+.2f}°")
                        print(f"🎯 TARGET: cx={cx} px | cy={cy} px | area={area} px² | "
                              f"size=({w}x{h})")

                    # 1. Расчет упреждения
                    now = time.time()
                    g = guidance.update(target, now, gimbal_attitude)

                    # DEBUG: вывод промежуточных значений из guidance
                    if is_debug_frame:
                        print(f"📊 GUIDANCE:")
                        print(f"   lead_deg=({guidance._smooth_lead_x:+.3f}, {guidance._smooth_lead_y:+.3f})°")
                        print(f"   lead_px=({g['lead_x']:+.2f}, {g['lead_y']:+.2f}) px")
                        print(f"   TTC={g['ttc']:.2f}s" if g['ttc'] is not None else "   TTC=None")
                        print(f"   phase={g['phase']} | N={g['current_N']:.2f}")
                        print(f"   LOS rate=({guidance.los.kf_cx.rate:+.3f}, {guidance.los.kf_cy.rate:+.3f}) deg/s")
                        print(f"   TTC growth_rate={guidance.ttc_est.kf.rate if guidance.ttc_est.kf.rate else 'None'}")
                        print(f"   TTC approaching={guidance.ttc_est._approaching}")

                    # 2. Абсолютные пиксельные координаты точки упреждения
                    aim_x = cx + g["lead_x"]
                    aim_y = cy + g["lead_y"]

                    # DEBUG: вывод финальных координат
                    if is_debug_frame:
                        print(f"🎯 AIM POINT: aim_x={aim_x:.1f} px | aim_y={aim_y:.1f} px")
                        print(f"   offset from center: ({aim_x - CENTER_X:+.1f}, {aim_y - CENTER_Y:+.1f}) px")
                        print(f"{'='*60}")

                    # 3. Ошибка для PD-регулятора
                    err_x = aim_x - CENTER_X
                    err_y = -(aim_y - CENTER_Y)

                    # 4. Рисуем рамку цели и желтый крестик упреждения
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    osd.draw_aim_point(frame, aim_x, aim_y)

                    # 5. Стандартный PD-регулятор
                    target_size = max(w, h, 1)
                    gain_scale = np.clip(cfg.REF_TARGET_SIZE / target_size, cfg.MIN_GAIN_SCALE, 1.0)
                    dt = max(now - prev_time, 1e-3)
                    raw_rate_roll = math.radians((cfg.KP * gain_scale) * err_x + (cfg.KD * gain_scale) * ((err_x - prev_err_x) / dt))
                    raw_rate_pitch = math.radians((cfg.KP * gain_scale) * err_y + (cfg.KD * gain_scale) * ((err_y - prev_err_y) / dt))
                    raw_rate_roll = np.clip(raw_rate_roll, -MAX_RATE_RAD_S, MAX_RATE_RAD_S)
                    raw_rate_pitch = np.clip(raw_rate_pitch, -MAX_RATE_RAD_S, MAX_RATE_RAD_S)
                    max_step = MAX_RATE_CMD_RATE * dt
                    rate_roll = prev_rate_roll + np.clip(raw_rate_roll - prev_rate_roll, -max_step, max_step)
                    rate_pitch = prev_rate_pitch + np.clip(raw_rate_pitch - prev_rate_pitch, -max_step, max_step)
                    prev_err_x, prev_err_y = err_x, err_y
                    prev_rate_roll, prev_rate_pitch = rate_roll, rate_pitch
                    prev_time = now

                    send_rc_override(master, rate_to_rc(rate_roll), rate_to_rc(rate_pitch), cfg.THRUST_RC, 1500, 1500)
                    osd.draw_tracking(
                        frame, target, err_x, err_y,
                        math.degrees(rate_roll), math.degrees(rate_pitch),
                        rate_to_rc(rate_roll), rate_to_rc(rate_pitch), cfg.THRUST_RC,
                        gain_scale, target_size, cfg.REF_TARGET_SIZE,
                        phase=g["phase"], ttc=g["ttc"],
                        current_N=g.get("current_N", cfg.N_NAV)
                    )
                else:
                    send_rc_override(master, 1500, 1500, cfg.THRUST_RC, 1500, 1500)
                    osd.draw_tracking_lost(frame)

            # --- ФАЗА MANUAL ---
            else:
                send_rc_override(master, ch1, ch2, ch3, ch4, ch5)
                osd.draw_manual(frame, ch1, ch2, ch3, ch4, target)

            cv2.imshow("ACRO PD-Tracker + Adaptive PN [DEBUG]", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    except Exception as e:
        print(f"❌ Ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n🛑 Остановка...")
        send_rc_override(master, 0, 0, 0, 0, 0)
        proc.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()