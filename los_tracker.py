from kalman import CVKalman1D

class LOSTracker:
    def __init__(self, cfg):
        self.cfg = cfg
        # Два независимых фильтра: один для X, другой для Y
        self.kf_cx = CVKalman1D(cfg.KF_ANGLE_PROCESS_NOISE, cfg.KF_ANGLE_MEASUREMENT_NOISE)
        self.kf_cy = CVKalman1D(cfg.KF_ANGLE_PROCESS_NOISE, cfg.KF_ANGLE_MEASUREMENT_NOISE)
        self._last_t = None
    
    def reset(self):
        self.kf_cx.reset(0.0)
        self.kf_cy.reset(0.0)
        self._last_t = None
    
    def update(self, cx, cy, t_now):
        dt = 0.0 if self._last_t is None else max(1e-4, t_now - self._last_t)
        dt = min(dt, self.cfg.MAX_KF_DT_SEC)
        self._last_t = t_now
        
        if dt > 0:
            self.kf_cx.predict(dt)
            self.kf_cy.predict(dt)
        
        self.kf_cx.update(cx)
        self.kf_cy.update(cy)
        
        return {
            "smoothed_cx": self.kf_cx.value,
            "smoothed_cy": self.kf_cy.value,
            "rate_x": self.kf_cx.rate,  # скорость в пикселях/сек
            "rate_y": self.kf_cy.rate,
        }