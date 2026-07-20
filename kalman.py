#!/usr/bin/env python3
"""
kalman.py
Один универсальный фильтр Калмана, модель "константная скорость".
Состояние: [x, x_dot] — сама величина и её скорость изменения.
"""
import numpy as np

class CVKalman1D:
    """
    Фильтр Калмана: состояние [x, x_dot], модель постоянной скорости.
    """
    def __init__(self, process_noise, measurement_noise, initial_x=0.0):
        self.x = np.array([initial_x, 0.0])  # [величина, скорость]
        self.P = np.eye(2) * 10.0            # ковариация
        self.q = process_noise
        self.r = measurement_noise
        self._initialized = False

    def reset(self, x0):
        """Сбросить фильтр на конкретное значение."""
        self.x = np.array([x0, 0.0])
        self.P = np.eye(2) * 10.0
        self._initialized = True

    def predict(self, dt):
        """Шаг предсказания."""
        if dt <= 0:
            return
        F = np.array([[1.0, dt],
                      [0.0, 1.0]])
        Q = self.q * np.array([[dt**4 / 4, dt**3 / 2],
                               [dt**3 / 2, dt**2]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, z):
        """Шаг коррекции."""
        if not self._initialized:
            self.reset(z)
            return
        
        H = np.array([1.0, 0.0])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.r
        K = (self.P @ H) / S
        
        self.x = self.x + K * y
        self.P = (np.eye(2) - np.outer(K, H)) @ self.P

    @property
    def value(self):
        """Текущая сглаженная оценка величины x."""
        return self.x[0]

    @property
    def rate(self):
        """Текущая сглаженная оценка скорости изменения x (x_dot)."""
        return self.x[1]