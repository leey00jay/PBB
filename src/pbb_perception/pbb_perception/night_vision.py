"""Lift night camera shadows before inference without changing sensor pixels."""
import cv2
import numpy as np


class NightVision:
    def __init__(self, enabled=False, gamma=.65):
        if not np.isfinite(gamma) or not .3 <= gamma <= 1.0:
            raise ValueError('night_gamma must be between 0.3 and 1.0')
        self.enabled = bool(enabled)
        self.gamma = float(gamma)
        self.lut = np.rint(255*(np.arange(256, dtype=float)/255)**gamma).astype(np.uint8)

    def apply(self, rgb):
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError('Night correction requires an RGB uint8 image')
        if not self.enabled or self.gamma == 1.0:
            return rgb.copy()
        # Lift value, preserving hue/saturation and existing white highlights.
        # A fixed LUT avoids frame-to-frame exposure pumping in wet reflections.
        hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
        hsv[:, :, 2] = cv2.LUT(hsv[:, :, 2], self.lut)
        return cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
