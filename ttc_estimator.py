#!/usr/bin/env python3
"""
TTC ESTIMATOR - Оценка времени до столкновения

Считает TTC по тому, как быстро растёт площадь рамки цели в кадре.
Формула: TTC = 2 / (d(lnA)/dt), где A — площадь рамки.

Использует фильтр Калмана для сглаживания ln(площади).
Гистерезис защищает от дребезга при переключении фазы "цель приближается".
"""
import math
from kalman import CVKalman1D


class TTCEstimator:
    """
    cfg — InterceptConfig из config.py
    """
    def __init__(self, cfg):
        self.cfg = cfg
        self.kf = CVKalman1D(
            cfg.KF_AREA_PROCESS_NOISE,
            cfg.KF_AREA_MEASUREMENT_NOISE
        )
        self._last_t = None
        
        # Состояние гистерезиса
        self._approaching = False
        self._opposite_streak = 0
    
    def reset(self):
        """Сброс состояния (при потере цели)."""
        self.kf.reset(0.0)
        self._last_t = None
        self._approaching = False
        self._opposite_streak = 0
    
    def update(self, area_px, t_now):
        """
        area_px — площадь рамки цели в пикселях (w*h)
        t_now — timestamp текущего кадра в секундах
        
        Возвращает dict:
          ttc — время до столкновения в секундах, либо None
          growth_rate — сглаженная d(lnA)/dt, для отладки
          smoothed_area — сглаженная площадь, для отладки
          approaching — текущее состояние гистерезиса
        """
        if area_px < self.cfg.MIN_AREA_PX:
            return {
                "ttc": None,
                "growth_rate": None,
                "smoothed_area": None,
                "approaching": self._approaching
            }
        
        ln_a = math.log(area_px)
        dt = 0.0 if self._last_t is None else max(1e-4, t_now - self._last_t)
        dt = min(dt, self.cfg.MAX_KF_DT_SEC)
        self._last_t = t_now
        
        if dt > 0:
            self.kf.predict(dt)
        self.kf.update(ln_a)
        
        growth_rate = self.kf.rate  # d(lnA)/dt
        smoothed_area = math.exp(self.kf.value)
        
        # Гистерезис: решаем, меняем ли состояние "приближается"
        raw_approaching = (
            growth_rate is not None and
            growth_rate > self.cfg.TTC_GROWTH_THRESHOLD
        )
        
        if raw_approaching == self._approaching:
            # Сырое измерение согласуется с текущим состоянием
            self._opposite_streak = 0
        else:
            # Сырое измерение расходится — копим streak
            self._opposite_streak += 1
            if self._opposite_streak >= self.cfg.TTC_HYSTERESIS_FRAMES:
                self._approaching = raw_approaching
                self._opposite_streak = 0
        
        if not self._approaching or growth_rate is None or growth_rate <= 0:
            return {
                "ttc": None,
                "growth_rate": growth_rate,
                "smoothed_area": smoothed_area,
                "approaching": self._approaching
            }
        
        ttc = 2.0 / growth_rate
        
        return {
            "ttc": ttc,
            "growth_rate": growth_rate,
            "smoothed_area": smoothed_area,
            "approaching": self._approaching
        }