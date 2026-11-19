"""Checkpoint scoring, reproducing the official judge (judge_module/judge.py).

The judge evaluates three conditions from the navigation pose at the moment
checkpoint_passed is received, and awards the LOWEST of their levels:
  distance   |pos - checkpoint.position|                      vs pos_bands
  centering  angle(camera +X, direction to the AR marker)      vs cen_bands
  roll       twist about the line of sight vs target attitude  vs roll_bands
"""
import math


def _rotate(v, q):
    """Same quaternion rotation formula as the judge (kept verbatim)."""
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    qx, qy, qz, qw = [c / n for c in q]
    vx, vy, vz = v
    t0 = qw * vx + qy * vz - qz * vy
    t1 = qw * vy - qx * vz + qz * vx
    t2 = qw * vz + qx * vy - qy * vx
    t3 = -qx * vx - qy * vy - qz * vz
    return [t0 * qw - t3 * qx - t1 * qz + t2 * qy,
            t1 * qw - t3 * qy + t0 * qz - t2 * qx,
            t2 * qw - t3 * qz - t0 * qy + t1 * qx]


def _unit(v):
    n = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    return [0.0, 0.0, 0.0] if n <= 1e-9 else [v[0] / n, v[1] / n, v[2] / n]


def _twist_deg(q, axis):
    n = math.sqrt(sum(c * c for c in q)) or 1.0
    qx, qy, qz, qw = [c / n for c in q]
    if qw < 0.0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw
    return math.degrees(2.0 * math.atan2(qx * axis[0] + qy * axis[1] + qz * axis[2], qw))


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def metrics(pos, quat, cp):
    """(distance [m], centering [deg], roll difference [deg]) as the judge sees it."""
    d = math.sqrt(sum((pos[i] - cp.position[i]) ** 2 for i in range(3)))
    view = _unit([cp.marker[i] - pos[i] for i in range(3)])
    fwd = _unit(_rotate([1.0, 0.0, 0.0], quat))
    dot = max(-1.0, min(1.0, sum(fwd[i] * view[i] for i in range(3))))
    centering = math.degrees(math.acos(dot))
    roll = abs(_wrap_deg(_twist_deg(quat, view) - _twist_deg(cp.quat, view)))
    return d, centering, roll


def _band_level(value, bands):
    for thr, lvl in bands:
        if value <= thr:
            return lvl
    return 0


def level(m, cp):
    """Judge level (0 = fail, 3 = best) for metrics m."""
    d, c, r = m
    lv = (_band_level(d, cp.pos_bands), _band_level(c, cp.cen_bands),
          _band_level(r, cp.roll_bands))
    return 0 if min(lv) == 0 else min(lv)


def _best_margin(bands):
    return max(bands, key=lambda b: b[1])[0]


def within(m, cp, frac):
    """True if every metric is below frac x its best-level (Lv3) margin."""
    d, c, r = m
    return (d <= frac * _best_margin(cp.pos_bands) and
            c <= frac * _best_margin(cp.cen_bands) and
            r <= frac * _best_margin(cp.roll_bands))
