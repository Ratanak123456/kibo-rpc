"""Reference trajectory for one leg: translate AND rotate at the same time.

The platform controller translates first and rotates afterwards with a fixed
40 % capacity profile. Here both run in parallel; their accelerations are
sized so the combined demand fits the fans (see fan_model), and the faster
channel is stretched to finish together with the slower one.
"""
import math

from kibo import config
from kibo.core.fan_model import IB2_INERTIA, IB2_MASS, cap_force, cap_torque
from kibo.core.geometry import (m_vec, q_axis_angle, q_conj, q_mul, q_normalize, q_rot_inv,
                                v_add, v_dot, v_mul, v_norm, v_sub)

_EPS_D = 1e-4     # [m]   below this a leg has no translation
_EPS_TH = 1e-3    # [rad] below this a leg has no rotation


def bangbang_time(x, vmax, amax):
    """Duration of a rest-to-rest trapezoid covering x."""
    if x <= 1e-9:
        return 0.0
    if x > vmax * vmax / amax:
        return x / vmax + vmax / amax
    return 2.0 * math.sqrt(x / amax)


def _accel_for_time(x, vmax, T):
    """Acceleration so that a trapezoid covers x in exactly T."""
    if x <= 1e-9 or T <= 1e-6:
        return 0.0
    if 2.0 * x / T <= vmax:
        return 4.0 * x / (T * T)
    return vmax / max(T - x / vmax, 1e-3)


class Profile(object):
    """Symmetric rest-to-rest trapezoid s(t) on [0, x]."""

    def __init__(self, x, vmax, amax):
        self.x = x
        if x <= 1e-9 or amax <= 0.0:
            self.T, self.tc, self.v, self.a = 0.0, 0.0, 0.0, 0.0
            return
        if x > vmax * vmax / amax:
            self.v, self.tc = vmax, vmax / amax
            self.T = x / vmax + self.tc
        else:
            self.tc = math.sqrt(x / amax)
            self.v = amax * self.tc
            self.T = 2.0 * self.tc
        self.a = amax

    def at(self, t):
        """(s, s_dot, s_ddot) at time t."""
        if self.T <= 0.0 or t >= self.T:
            return self.x, 0.0, 0.0
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        a, tc, v = self.a, self.tc, self.v
        if t < tc:
            return 0.5 * a * t * t, a * t, a
        if t < self.T - tc:
            return 0.5 * a * tc * tc + v * (t - tc), v, 0.0
        tg = self.T - t
        return self.x - 0.5 * a * tg * tg, a * tg, -a


class LegPlan(object):
    """Reference from pose (p0, q0) to (p1, q1) using `eff` of the fan capacity.

    ref(t) -> (p, v, a, q, w_body, alpha_body); after T it holds (p1, q1).
    Translation follows the straight line p0->p1, rotation a fixed body axis.
    """

    def __init__(self, p0, q0, p1, q1, eff,
                 v_max=config.TRAJ_V_MAX, w_max=config.TRAJ_W_MAX):
        self.p0, self.p1 = list(p0), list(p1)
        dr = v_sub(p1, p0)
        self.d = v_norm(dr)
        self.u = v_mul(dr, 1.0 / self.d) if self.d > 1e-6 else [1.0, 0.0, 0.0]
        self.q0, self.q1 = q_normalize(q0), q_normalize(q1)
        dq = q_mul(q_conj(self.q0), self.q1)
        if dq[3] < 0.0:
            dq = [-c for c in dq]
        self.th = 2.0 * math.acos(max(-1.0, min(1.0, dq[3])))
        sn = v_norm(dq[:3])
        self.axis = v_mul(dq[:3], 1.0 / sn) if sn > 1e-9 else [1.0, 0.0, 0.0]
        moves, turns = self.d > _EPS_D, self.th > _EPS_TH

        # force capacity along the travel direction at the weakest attitude
        # met during the rotation; torque capacity about the rotation axis
        cf = 1.0
        if moves:
            cf = min(eff * cap_force(q_rot_inv(self._q_at(self.th * s), self.u))
                     for s in (0.0, 0.25, 0.5, 0.75, 1.0))
        ct = eff * cap_torque(self.axis)
        i_ax = v_dot(self.axis, m_vec(IB2_INERTIA, self.axis))

        # split the fan budget between translation (lam) and rotation (1 - lam)
        if moves and turns:
            lams = config.TRAJ_LAMBDAS
        else:
            lams = [1.0] if moves else [0.0]
        best = None
        for lam in lams:
            at = lam * cf / IB2_MASS
            ar = (1.0 - lam) * ct / i_ax
            tt = bangbang_time(self.d, v_max, at) if moves else 0.0
            tr = bangbang_time(self.th, w_max, ar) if turns else 0.0
            if best is None or max(tt, tr) < best[0]:
                best = (max(tt, tr), at, ar)
        T, at, ar = best
        # stretch the faster channel to finish together (frees capacity)
        if moves and T > 0:
            at = min(at, _accel_for_time(self.d, v_max, T))
        if turns and T > 0:
            ar = min(ar, _accel_for_time(self.th, w_max, T))
        self.pt = Profile(self.d, v_max, at)
        self.pr = Profile(self.th, w_max, ar)
        self.T = max(self.pt.T, self.pr.T)

    def _q_at(self, angle):
        return q_mul(self.q0, q_axis_angle(self.axis, angle))

    def ref(self, t):
        if t >= self.T:
            zero = [0.0, 0.0, 0.0]
            return list(self.p1), zero, zero, list(self.q1), zero, zero
        s, sd, sdd = self.pt.at(t)
        th, thd, thdd = self.pr.at(t)
        return (v_add(self.p0, v_mul(self.u, s)), v_mul(self.u, sd), v_mul(self.u, sdd),
                self._q_at(th), v_mul(self.axis, thd), v_mul(self.axis, thdd))


def hold_plan(pos, quat):
    """A plan that simply holds a pose."""
    return LegPlan(pos, quat, pos, quat, config.WRENCH_EFF)
