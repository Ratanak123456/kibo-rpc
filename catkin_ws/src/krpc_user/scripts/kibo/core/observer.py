"""Command history and disturbance observer.

The web simulator applies an unknown force (the rulebook's "microgravity
module": ~10-20 mN wandering with 45-95 mN spikes) on top of airflow and
ISS-frame drift. The observer estimates it as

    d_hat = (v(t) - v(t - W) - integral of commanded fan accel) / W

over a W = 1 s window of measured velocity, then low-pass filters it. Both
the observer and the nav-delay predictor need the command history.
"""
from kibo import config
from kibo.core.geometry import v_add, v_clamp, v_mul, v_sub


class CommandHistory(object):
    """Piecewise-constant commanded accelerations: (t, a_world, alpha_body)."""

    def __init__(self, keep_sec):
        self.keep_sec = keep_sec
        self.items = []

    def append(self, t, a_world, alpha_body):
        self.items.append((t, a_world, alpha_body))
        cut = t - self.keep_sec
        while self.items and self.items[0][0] < cut:
            self.items.pop(0)

    def first_time(self):
        return self.items[0][0] if self.items else None

    def integrate_accel(self, t0, t1):
        """Integral of the commanded world acceleration over [t0, t1]."""
        out = [0.0, 0.0, 0.0]
        hs = self.items
        for i in range(len(hs)):
            ts = hs[i][0]
            te = hs[i + 1][0] if i + 1 < len(hs) else max(t1, ts)
            lo, hi = max(ts, t0), min(te, t1)
            if hi > lo:
                out = v_add(out, v_mul(hs[i][1], hi - lo))
        return out


class DisturbanceObserver(object):
    """Estimates the unknown external acceleration [m/s^2, world frame]."""

    def __init__(self, window=config.OBS_WIN, gain=config.OBS_GAIN, limit=config.OBS_D_LIM):
        self.window = window
        self.gain = gain
        self.limit = limit
        self.d_hat = [0.0, 0.0, 0.0]
        self._vhist = []        # (t_meas, v_meas_world)

    def update(self, t_meas, v, history):
        """Feed one measured velocity; history = CommandHistory of commands."""
        self._vhist.append((t_meas, list(v)))
        while self._vhist and self._vhist[0][0] < t_meas - self.window - 0.3:
            self._vhist.pop(0)
        old = None
        for tv in self._vhist:
            if tv[0] <= t_meas - self.window:
                old = tv
            else:
                break
        first = history.first_time()
        if old is None or first is None or first > old[0]:
            return
        span = t_meas - old[0]
        if span < 0.5 * self.window:
            return
        dv = v_sub(v, old[1])
        fan = history.integrate_accel(old[0], t_meas)
        est = v_mul(v_sub(dv, fan), 1.0 / span)
        g = self.gain
        self.d_hat = v_clamp(v_add(v_mul(self.d_hat, 1.0 - g), v_mul(est, g)), self.limit)
