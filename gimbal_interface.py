#!/usr/bin/env python3
"""
GIMBAL INTERFACE — мост между Python и виртуальным гимбалом ArduPilot.
"""
import math
import time
from pymavlink import mavutil

GIMBAL_MANAGER_FLAGS_NONE = 0 # битовая мака для гимбал 0 - поведение по умолчаеию

class GimbalInterface:
    def __init__(self, master, telemetry_rate_hz=100):
        self.master = master
        self._last_attitude = {
            "pitch_deg": 0.0,
            "yaw_deg": 0.0,
            "roll_deg": 0.0,
            "timestamp": 0.0,
        }
        self._last_cmd_sent = 0.0
        self._cmd_interval_sec = 1.0 / 30.0  # отправляем команды не чаще 30 Гц

        # Запрашиваем телеметрию гимбала с нужной частотой
        self._set_telemetry_rate(telemetry_rate_hz)

    def _set_telemetry_rate(self, rate_hz):
        """Устанавливает интервал отправки GIMBAL_DEVICE_ATTITUDE_STATUS."""
        interval_us = int(1_000_000 / rate_hz)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,  # <-- ИСПРАВЛЕНО ЗДЕСЬ
            interval_us,
            0, 0, 0, 0, 0,
        )
        print(f"📡 Запрошена телеметрия гимбала: {rate_hz} Гц (интервал {interval_us} мкс)")

    def update(self):
        """Читает все доступные MAVLink-сообщения и обновляет состояние гимбала."""
        while True:
            msg = self.master.recv_match(
                type="GIMBAL_DEVICE_ATTITUDE_STATUS", blocking=False
            )
            if msg is None:
                break

            q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]  # [w, x, y, z]
            roll, pitch, yaw = self._quaternion_to_euler(q)

            self._last_attitude = {
                "pitch_deg": math.degrees(pitch),
                "yaw_deg": math.degrees(yaw),
                "roll_deg": math.degrees(roll),
                "timestamp": time.time(),
            }

    def set_aim(self, pitch_deg, yaw_deg, roll_deg=0.0):
        """Отправляет команду гимбалу: "повернись на указанные углы в мировой системе"."""
        now = time.time()
        if now - self._last_cmd_sent < self._cmd_interval_sec:
            return

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            0,
            GIMBAL_MANAGER_FLAGS_NONE,
            pitch_deg,
            yaw_deg,
            roll_deg,
            0,  # gimbal_device_id (0 = default)
            0, 0,
        )
        self._last_cmd_sent = now

    def get_attitude(self):
        """Возвращает последние скомпенсированные углы гимбала."""
        return self._last_attitude.copy()

    @staticmethod
    def _quaternion_to_euler(q):
        """Конвертация quaternion [w, x, y, z] → euler angles (roll, pitch, yaw) в радианах."""
        w, x, y, z = q

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw