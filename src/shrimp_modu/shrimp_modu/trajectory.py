#!/usr/bin/env python3

import numpy as np


WAYPOINTS = np.array([
    [2.0, 0.0],
    [1.84775907, 0.76536686],
    [1.41421356, 1.41421356],
    [0.76536686, 1.84775907],
    [0.0, 2.0],
    [-0.76536686, 1.84775907],
    [-1.41421356, 1.41421356],
    [-1.84775907, 0.76536686],
    [-2.0, 0.0],
    [-1.84775907, -0.76536686],
    [-1.41421356, -1.41421356],
    [-0.76536686, -1.84775907],
    [0.0, -2.0],
    [0.76536686, -1.84775907],
    [1.41421356, -1.41421356],
    [1.84775907, -0.76536686],
    [2.0, 0.0],
], dtype=float)


def build_reference_path(waypoints=None, samples_per_segment=50):
    pts = WAYPOINTS if waypoints is None else np.asarray(waypoints, dtype=float)
    path_x = []
    path_y = []

    for i in range(len(pts) - 1):
        p0 = pts[i]
        p1 = pts[i + 1]
        t = np.linspace(0.0, 1.0, samples_per_segment)
        path_x.extend(p0[0] + (p1[0] - p0[0]) * t)
        path_y.extend(p0[1] + (p1[1] - p0[1]) * t)

    return pts, np.array(path_x), np.array(path_y)
