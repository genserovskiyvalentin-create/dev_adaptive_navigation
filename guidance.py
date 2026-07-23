import math
from los_tracker import LOSTracker
from ttc_estimator import TTCEstimator

class PhaseState:
    ALIGN = "ALIGN"
    TERMINAL = "TERMINAL"

class InterceptGuidance1:
    def __init__(self, cfg):
        self.cfg = cfg
        self.los = LOSTracker(cfg)
        self.ttc_est = TTCEstimator(cfg)
        self._frozen = False
        self._frozen_lead = None
        self.phase = PhaseState.ALIGN
        self._debug_counter = 0

        self.current_N = cfg.N_NAV
        self._last_N_t = None

        self._smooth_lead_x = 0.0
        self._smooth_lead_y = 0.0
        self.LEAD_SMOOTHING_ALPHA = 0.15

        # Фокусное расстояние в пикселях (pinhole-модель) для перевода px <-> deg
        self._fx = cfg.FRAME_W / (2.0 * math.tan(math.radians(cfg.FOV_H_DEG) / 2.0))
        
        # --- Захват упреждения (Lead Capture) ---
        # Состояние для включения обратной связи только после захвата точки упреждения
        self._lead_capture_active = False  # Флаг: обратная связь упреждения включена
        self._capture_counter = 0  # Счётчик кадров в области захвата
        self._initial_lead_set = False  # Флаг: начальное упреждение установлено
        self._prev_dist_to_lead = None  # Для вычисления производной расстояния
        self._dist_derivative = 0.0  # Производная расстояния (фильтрованная)

    def reset(self):
        self.los.reset()
        self.ttc_est.reset()
        self._frozen = False
        self._frozen_lead = None
        self.phase = PhaseState.ALIGN
        self.current_N = self.cfg.N_NAV
        self._last_N_t = None
        self._smooth_lead_x = 0.0
        self._smooth_lead_y = 0.0
        # Сброс состояния захвата упреждения
        self._lead_capture_active = False
        self._capture_counter = 0
        self._initial_lead_set = False
        self._prev_dist_to_lead = None
        self._dist_derivative = 0.0
        self._initial_capture_dist = None
        self._min_dist_since_start = None

    def _target_size_ratio(self, bbox_w):
        return bbox_w / float(self.cfg.FRAME_W)

    def _pixel_to_world_angle(self, cx, cy, gimbal_attitude):
        """
        Переводит пиксельные координаты цели в кадре в угол линии визирования
        в МИРОВОЙ системе координат, компенсируя текущую ориентацию гимбала
        (yaw/pitch/roll), чтобы вращение носителя не принималось за движение цели.
        """
        dx = cx - self.cfg.FRAME_W / 2.0
        dy = cy - self.cfg.FRAME_H / 2.0
        if self.cfg.CAMERA_FLIP_Y:
            dy = -dy

        # Компенсация крена: разворачиваем офсет в "горизонт" гимбала,
        # иначе при ненулевом roll оси az/el будут перепутаны.
        roll_rad = math.radians(gimbal_attitude.get("roll_deg", 0.0))
        cos_r, sin_r = math.cos(-roll_rad), math.sin(-roll_rad)
        dx_r = dx * cos_r - dy * sin_r
        dy_r = dx * sin_r + dy * cos_r

        az_offset_deg = math.degrees(math.atan(dx_r / self._fx))
        el_offset_deg = math.degrees(math.atan(dy_r / self._fx))

        world_az_deg = gimbal_attitude.get("yaw_deg", 0.0) + az_offset_deg
        world_el_deg = gimbal_attitude.get("pitch_deg", 0.0) + el_offset_deg
        return world_az_deg, world_el_deg

    def update(self, target, t_now, gimbal_attitude):
        """
        gimbal_attitude — dict из GimbalInterface.get_attitude():
            {"pitch_deg":..., "yaw_deg":..., "roll_deg":..., "timestamp":...}
        """
        cx, cy, area, (x, y, w, h) = target
        size_ratio = self._target_size_ratio(w)

        # --- ЛОС теперь считается по мировому углу, а не по сырым пикселям ---
        world_az_deg, world_el_deg = self._pixel_to_world_angle(cx, cy, gimbal_attitude)
                # DEBUG: проверка компенсации гимбала
        if self._debug_counter % 30 == 0:
            print(f"[COMPENSATION] raw_px=({cx},{cy}) | "
                  f"gimbal=({gimbal_attitude['roll_deg']:+.1f}°, "
                  f"{gimbal_attitude['pitch_deg']:+.1f}°, "
                  f"{gimbal_attitude['yaw_deg']:+.1f}°) | "
                  f"world=({world_az_deg:+.1f}°, {world_el_deg:+.1f}°)")
        
        
        los_out = self.los.update(world_az_deg, world_el_deg, t_now)
        ttc_out = self.ttc_est.update(area, t_now)

        rate_x = los_out["rate_x"]  # теперь deg/s в мировой СК, а не px/s
        rate_y = los_out["rate_y"]
        ttc = ttc_out["ttc"]

        dt_n = 0.0 if self._last_N_t is None else max(1e-4, t_now - self._last_N_t)
        self._last_N_t = t_now

        los_rate_mag = math.hypot(rate_x, rate_y)  # deg/s

        # --- ЛОГИКА ЗАХВАТА УПРЕЖДЕНИЯ (Lead Capture) ---
        # Решение бага: при запуске дрона обратная связь упреждения ОТКЛЮЧЕНА
        # Дрон просто летит к расчётной точке упреждения без контроля угловой скорости цели
        # Обратная связь включается ТОЛЬКО когда дрон ФИЗИЧЕСКИ повернулся к точке упреждения
        # Это гарантирует, что нос дрона перелетит переднюю часть цели перед включением ОС
        
        # Вычисляем точку упреждения для проверки попадания в область захвата
        temp_lead_az_deg, temp_lead_el_deg, temp_lead_mag = self._compute_lead(rate_x, rate_y, ttc)
        temp_lead_px_x = self._fx * math.tan(math.radians(temp_lead_az_deg))
        temp_lead_px_y = self._fx * math.tan(math.radians(temp_lead_el_deg))
        if self.cfg.CAMERA_FLIP_Y:
            temp_lead_px_y = -temp_lead_px_y
        
        # Абсолютные координаты точки упреждения в кадре
        lead_point_x = cx + temp_lead_px_x
        lead_point_y = cy + temp_lead_px_y
        
        # Расстояние от ЦЕНТРА КАДРА до точки упреждения
        frame_center_x = self.cfg.FRAME_W / 2.0
        frame_center_y = self.cfg.FRAME_H / 2.0
        dist_to_lead = math.hypot(lead_point_x - frame_center_x, lead_point_y - frame_center_y)
        
        # Инициализация при первом кадре или после сброса
        if self._prev_dist_to_lead is None:
            self._prev_dist_to_lead = dist_to_lead
            self._dist_derivative = 0.0
            self._initial_capture_dist = dist_to_lead  # Запоминаем начальное расстояние
            self._min_dist_since_start = dist_to_lead  # Минимальное расстояние с начала
        
        # Производная расстояния (отрицательная = центр кадра приближается к точке упреждения)
        self._dist_derivative = 0.7 * self._dist_derivative + 0.3 * (dist_to_lead - self._prev_dist_to_lead)
        self._prev_dist_to_lead = dist_to_lead
        
        # Отслеживаем минимальное достигнутое расстояние (для гарантии разворота)
        self._min_dist_since_start = min(self._min_dist_since_start, dist_to_lead)
        
        # УСЛОВИЕ ЗАХВАТА (строгое):
        # 1. Точка упреждения находится в расширенной зоне вокруг центра кадра
        # 2. Дрон ФИЗИЧЕСКИ приблизился к точке упреждения (расстояние уменьшилось от начального)
        # 3. Производная отрицательная или близка к нулю (движение продолжается или стабилизировалось)
        in_capture_zone = dist_to_lead <= self.cfg.LEAD_CAPTURE_RADIUS_PX * 2  # Расширенная зона (16px)
        moved_toward_lead = self._min_dist_since_start < self._initial_capture_dist * 0.85  # Уменьшилось на 15%+
        not_moving_away = self._dist_derivative < 1.0  # Не удаляемся быстро
        
        should_capture = in_capture_zone and moved_toward_lead and not_moving_away
        
        if not self._lead_capture_active:
            # Фаза захвата: обратная связь упреждения ОТКЛЮЧЕНА
            # Дрон просто летит к точке упреждения без контроля угловой скорости цели
            if should_capture:
                self._capture_counter += 1
                if self._capture_counter >= self.cfg.LEAD_CAPTURE_FRAMES:
                    # Захват завершён: включаем обратную связь упреждения
                    # К этому моменту нос дрона гарантированно перелетел переднюю часть цели
                    self._lead_capture_active = True
                    self._initial_lead_set = True
                    if self._debug_counter % 10 == 0:
                        print(f"[LEAD CAPTURE] ✅ Захват выполнен! Обратная связь упреждения ВКЛЮЧЕНА")
            else:
                # Центр кадра ещё не начал движение к точке упреждения или вышел из зоны
                # Сбрасываем счётчик постепенно для гистерезиса
                self._capture_counter = max(0, self._capture_counter - 1)
                
            # В фазе захвата НЕ обновляем N на основе LOS-скорости цели
            # (обратная связь упреждения отключена - дрон просто летит к точке)
            if self._debug_counter % 30 == 0 and not self._lead_capture_active:
                print(f"[LEAD CAPTURE] 🎯 Захват: {self._capture_counter}/{self.cfg.LEAD_CAPTURE_FRAMES} | "
                      f"dist={dist_to_lead:.1f}px | min_dist={self._min_dist_since_start:.1f} | "
                      f"init_dist={self._initial_capture_dist:.1f} | deriv={self._dist_derivative:.2f} | "
                      f"in_zone={in_capture_zone} | moved={moved_toward_lead} | not_away={not_moving_away}")
        else:
            # Фаза сопровождения: обратная связь упреждения ВКЛЮЧЕНА
            # Теперь контролируем угловую скорость цели (стремимся к нулю)
            # Адаптация N работает как обычно
            if los_rate_mag > self.cfg.LOS_OVERSHOOT_DEG_S:
                self.current_N -= self.cfg.N_ADAPT_RATE * 5.0 * dt_n
            elif los_rate_mag > self.cfg.LOS_DEADZONE_DEG_S:
                if self.current_N < self.cfg.N_MAX:
                    self.current_N += self.cfg.N_ADAPT_RATE * los_rate_mag * dt_n
            else:
                self.current_N -= self.cfg.N_ADAPT_RATE * 0.5 * dt_n

        self.current_N = max(self.cfg.N_MIN, min(self.cfg.N_MAX, self.current_N))

        if size_ratio >= self.cfg.FREEZE_SIZE_RATIO:
            if not self._frozen:
                self._frozen = True
                self._frozen_lead = self._compute_lead(rate_x, rate_y, ttc)
            lead_az_deg, lead_el_deg, lead_mag = self._frozen_lead
        else:
            self._frozen = False
            lead_az_deg, lead_el_deg, lead_mag = self._compute_lead(rate_x, rate_y, ttc)

        self._smooth_lead_x += self.LEAD_SMOOTHING_ALPHA * (lead_az_deg - self._smooth_lead_x)
        self._smooth_lead_y += self.LEAD_SMOOTHING_ALPHA * (lead_el_deg - self._smooth_lead_y)

        self.phase = PhaseState.TERMINAL if ttc is not None else PhaseState.ALIGN

        # --- Перевод углового упреждения обратно в пиксели камеры для OSD/PD ---
        lead_px_x = self._fx * math.tan(math.radians(self._smooth_lead_x))
        lead_px_y = self._fx * math.tan(math.radians(self._smooth_lead_y))
        if self.cfg.CAMERA_FLIP_Y:
            lead_px_y = -lead_px_y

        lead_px_mag = math.hypot(lead_px_x, lead_px_y)
        if lead_px_mag > self.cfg.MAX_LEAD_PX and lead_px_mag > 0:
            scale = self.cfg.MAX_LEAD_PX / lead_px_mag
            lead_px_x *= scale
            lead_px_y *= scale

        self._debug_counter += 1
        if self._debug_counter % 30 == 0:
            ttc_str = f"{ttc:.2f}" if ttc is not None else "None"
            flag = " [N DROP!]" if los_rate_mag > self.cfg.LOS_OVERSHOOT_DEG_S else ""
            capture_status = "CAPTURE" if not self._lead_capture_active else "TRACK"
            print(f"[GUIDANCE] az={world_az_deg:.1f}, el={world_el_deg:.1f} | "
                  f"rate={los_rate_mag:.2f} deg/s | TTC={ttc_str}{flag} | "
                  f"N={self.current_N:.2f} | lead_deg=({self._smooth_lead_x:.2f},{self._smooth_lead_y:.2f}) | "
                  f"Phase={self.phase} | Capture={capture_status}")

        return {
            "lead_x": lead_px_x,
            "lead_y": lead_px_y,
            "ttc": ttc,
            "phase": self.phase,
            "current_N": self.current_N,
            "lead_capture_active": self._lead_capture_active,
        }

    def _compute_lead(self, rate_x, rate_y, ttc):
        """rate_x/rate_y — deg/s. Возвращает упреждение В ГРАДУСАХ."""
        if ttc is None:
            return 0.0, 0.0, 0.0

        t_go = min(ttc, self.cfg.TTC_CAP_SEC)
        lead_az_deg = self.current_N * rate_x * t_go
        lead_el_deg = self.current_N * rate_y * t_go

        lead_mag = math.hypot(lead_az_deg, lead_el_deg)
        if lead_mag > self.cfg.MAX_LEAD_DEG and lead_mag > 0:
            scale = self.cfg.MAX_LEAD_DEG / lead_mag
            lead_az_deg *= scale
            lead_el_deg *= scale

        return lead_az_deg, lead_el_deg, lead_mag