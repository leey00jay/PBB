"""Synthetic lane masks shared by the part-level and runtime tests."""
import numpy as np


class SyntheticDetector:
    def __init__(self, left=True, right=True, width=3.5):
        self.cut_v = 0
        x, y = np.meshgrid(np.linspace(4.5, 25, 80), np.linspace(-4, 4, 161), indexing='ij')
        self.grid = np.column_stack((x.ravel(), y.ravel()))
        self.left = (np.abs(y - width/2) < .06).astype(float)*.9 if left else np.zeros_like(x)
        self.right = (np.abs(y + width/2) < .06).astype(float)*.9 if right else np.zeros_like(x)
        self.rgb = np.zeros((*x.shape,3),np.uint8)

    def detect(self, rgb):
        return np.zeros_like(self.left), self.left, self.right
