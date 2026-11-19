"""Wrench controller: reference + navigation -> body force and torque.

* position: PID on world-frame error + planned acceleration
* attitude: quaternion PD (same law as the platform ctl) + planned alpha
* nav arrives ~0.1 s late: the state is predicted forward with the commands
* disturbance observer feeds its estimate forward
* priority saturation: when the fans run out, keep feedback + disturbance
  rejection and scale the planned (feedforward) part down; the caller slows
  the reference accordingly (time-warp)
"""
import math

from kibo import config
from kibo.core.fan_model import (FAN_FMAX, IB2_INERTIA, IB2_INERTIA_DIAG_INV, IB2_MASS,
                                 max_fan)
from kibo.core.geometry import (m_vec, q_axis_angle, q_conj, q_mul, q_normalize, q_rot,
                                q_rot_inv, v_add, v_clamp, v_cross, v_mul, v_norm, v_sub)
from kibo.core.observer import CommandHistory, DisturbanceObserver


class ControlOutput(object):
    """Result of one control step."""
    __slots__ = ('force', 'torque', 'pos_err', 'att_err_deg', 'err_vec')

    def __init__(self, force, torque, pos_err, att_err_deg, err_vec):
        self.force = force              # [N]  body frame
        self.torque = torque            # [Nm] body frame
        self.pos_err = pos_err          # [m]  |reference - predicted position|
        self.att_err_deg = att_err_deg  # [deg]
        self.err_vec = err_vec          # [m]  reference - position (world)


class WrenchController(object):

    def __init__(self):
        self.kp, self.kd, self.ki = config.CTRL_KP, config.CTRL_KD, config.CTRL_KI
        self.kpa, self.kda = config.CTRL_KPA, config.CTRL_KDA
        self.fan_limit = config.CTRL_FAN_LIMIT_FRAC * FAN_FMAX
        self.history = CommandHistory(max(config.CTRL_PRED_MAX, config.OBS_WIN) + 0.6)
        self.observer = DisturbanceObserver()
        self.integ = [0.0, 0.0, 0.0]
        self.last_t = None
        self.saturation = 1.0       # <1: command was scaled to fit the fans
        self.ff_scale = 1.0         # share of feedforward that fitted

    @property
    def d_hat(self):
        """Estimated disturbance acceleration [m/s^2, world]."""
        return self.observer.d_hat

    def disturbance_force(self):
        """Estimated disturbance magnitude [N]."""
        return v_norm(self.observer.d_hat) * IB2_MASS

    def reset_integrator(self):
        self.integ = [0.0, 0.0, 0.0]

    # ------------------------------------------------------------------ step
    def step(self, t_now, t_meas, p, v, q, w, ref, rate=1.0):
        """One update.

        t_now   current time; t_meas: timestamp of the nav sample
        p, v    world position / velocity;  q, w: attitude / body angular rate
        ref     LegPlan.ref(tau) = (p, v, a, q, w_body, alpha_body)
        rate    d(tau)/dt of the time-warped reference (scales v, a, w, alpha)
        """
        p_r, v_r, a_r, q_r, w_r, al_r = ref
        dt = 0.0 if self.last_t is None else max(0.0, min(0.1, t_now - self.last_t))
        self.last_t = t_now
        self.observer.update(t_meas, v, self.history)
        p, v, q, w = self._predict(t_now, t_meas, p, v, q, w)
        r2 = rate * rate
        v_r, a_r = v_mul(v_r, rate), v_mul(a_r, r2)
        w_r, al_r = v_mul(w_r, rate), v_mul(al_r, r2)

        F_base, F_ff, e = self._translation(p, v, q, p_r, v_r, a_r, dt)
        T_base, T_ff, qe, Iw = self._attitude(q, w, q_r, w_r, al_r)
        F, T = self._fit_to_fans(F_base, F_ff, T_base, T_ff)

        a_app = q_rot(q, v_mul(F, 1.0 / IB2_MASS))
        al_app = m_vec(IB2_INERTIA_DIAG_INV, v_sub(T, v_cross(w, Iw)))
        self.history.append(t_now, a_app, al_app)
        att_err = 2.0 * math.degrees(math.asin(min(1.0, v_norm(qe[:3]))))
        return ControlOutput(F, T, v_norm(e), att_err, e)

    # ------------------------------------------------------------- internals
    def _translation(self, p, v, q, p_r, v_r, a_r, dt):
        e = v_sub(p_r, p)
        ev = v_sub(v_r, v)
        if v_norm(e) < config.CTRL_I_ZONE:
            self.integ = v_clamp(v_add(self.integ, v_mul(e, dt)), config.CTRL_I_LIM / self.ki)
        afb = v_add(v_add(v_mul(e, self.kp), v_mul(ev, self.kd)), v_mul(self.integ, self.ki))
        afb = v_clamp(afb, config.CTRL_A_FB_MAX)
        a_base = v_sub(afb, self.observer.d_hat)
        F_base = q_rot_inv(q, v_mul(a_base, IB2_MASS))
        F_ff = q_rot_inv(q, v_mul(a_r, IB2_MASS))
        return F_base, F_ff, e

    def _attitude(self, q, w, q_r, w_r, al_r):
        qe = q_mul(q_conj(q_r), q)
        if qe[3] < 0.0:
            qe = [-c for c in qe]
        w_r_b = q_rot_inv(qe, w_r)
        al_r_b = q_rot_inv(qe, al_r)
        wc = v_add(v_mul(qe[:3], -2.0 * self.kpa / self.kda), w_r_b)
        Iw = m_vec(IB2_INERTIA, w)
        T_base = v_add(m_vec(IB2_INERTIA, v_mul(v_sub(wc, w), self.kda)), v_cross(w, Iw))
        T_ff = m_vec(IB2_INERTIA, al_r_b)
        return T_base, T_ff, qe, Iw

    def _fit_to_fans(self, F_base, F_ff, T_base, T_ff):
        lim = self.fan_limit
        F, T = v_add(F_base, F_ff), v_add(T_base, T_ff)
        s_ff = 1.0
        if max_fan(F, T) > lim:
            base = max_fan(F_base, T_base)
            if base > lim:
                s_ff = 0.0
                k = lim / base
                F, T = v_mul(F_base, k), v_mul(T_base, k)
            else:
                lo, hi = 0.0, 1.0
                for _ in range(config.CTRL_SAT_BISECT):
                    mid = 0.5 * (lo + hi)
                    if max_fan(v_add(F_base, v_mul(F_ff, mid)),
                               v_add(T_base, v_mul(T_ff, mid))) <= lim:
                        lo = mid
                    else:
                        hi = mid
                s_ff = lo
                F = v_add(F_base, v_mul(F_ff, s_ff))
                T = v_add(T_base, v_mul(T_ff, s_ff))
        self.ff_scale = s_ff
        self.saturation = min(1.0, lim / max(max_fan(F, T), 1e-9))
        return F, T

    def _predict(self, t_now, t_meas, p, v, q, w):
        """Propagate the delayed nav sample to t_now with the commanded accels."""
        age = max(0.0, min(config.CTRL_PRED_MAX, t_now - t_meas))
        hs = self.history.items
        if age <= 1e-4 or not hs:
            return p, v, q, w
        t0 = t_now - age
        dp, dv, dw = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
        for i in range(len(hs)):
            ts, a, al = hs[i]
            a = v_add(a, self.observer.d_hat)
            te = hs[i + 1][0] if i + 1 < len(hs) else t_now
            lo, hi = max(ts, t0), min(te, t_now)
            if hi <= lo:
                continue
            h = hi - lo
            dp = v_add(dp, v_add(v_mul(dv, h), v_mul(a, 0.5 * h * h)))
            dv = v_add(dv, v_mul(a, h))
            dw = v_add(dw, v_mul(al, h))
        p2 = v_add(v_add(p, v_mul(v, age)), dp)
        wm = v_add(w, v_mul(dw, 0.5))
        ang = v_norm(wm) * age
        q2 = q_mul(q, q_axis_angle(v_mul(wm, 1.0 / v_norm(wm)), ang)) if ang > 1e-9 else q
        return p2, v_add(v, dv), q_normalize(q2), v_add(w, dw)
