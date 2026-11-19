"""Custom guidance through /ctl/wrench (v3): the node drives the fans itself.

Per leg: plan a parallel translate+rotate trajectory (core.trajectory), track
it with the wrench controller (core.controller) on every fresh nav sample,
slow the reference down whenever the robot lags (time-warp), then report the
checkpoint as soon as the pose is verified - no 3 s ctl hold.

Raises GuidanceFallback on a gross, persistent failure; the runner then
zeroes the fans and continues with the platform ctl.
"""
import math
import threading
import time

from kibo import config
from kibo.core import judge
from kibo.core.controller import WrenchController
from kibo.core.geometry import v_dot, v_mul, v_norm, v_sub
from kibo.core.trajectory import LegPlan, hold_plan
from kibo.guidance.base import Guidance, GuidanceFallback
from kibo.ros_io import log
from kibo.ros_io.wrench_out import WrenchOutput


def _ramp_down(x, start, stop):
    """1 below start, 0 above stop, linear in between."""
    return max(0.0, min(1.0, (stop - x) / (stop - start)))


class WrenchGuidance(Guidance):
    name = 'WRENCH (v3)'

    def __init__(self, ctx):
        Guidance.__init__(self, ctx)
        self.output = WrenchOutput()
        self.ctrl = WrenchController()
        self.engaged = False
        self._dist_peak = 0.0
        self._diag_t = -1e9
        self._diag = []

    # ------------------------------------------------------------ selection
    def available(self):
        """True if the platform ctl is idle (STAND_BY) and nav is flowing."""
        if not config.USE_WRENCH:
            return False
        state = self.ctx.state
        wall_end = time.time() + config.MODE_DECISION_WAIT
        while self.ctx.alive() and time.time() < wall_end:
            if state.ctl_type is not None and state.nav is not None:
                break
            time.sleep(0.02)
        ok = (state.ctl_type == config.CTL_STAND_BY and state.nav is not None and
              state.nav.stamp is not None)
        log.info('ctl status type=%s nav=%s -> guidance: %s', state.ctl_type,
                 'ok' if state.nav is not None else 'none',
                 self.name if ok else 'CTL (v2.1)')
        self.engaged = ok
        return ok

    # ------------------------------------------------------------- planning
    def _plan(self, p0, q0, p1, q1):
        strong = self._dist_peak > config.STRONG_DIST_N
        return LegPlan(p0, q0, p1, q1, config.STRONG_DIST_EFF if strong else config.WRENCH_EFF)

    def estimate(self, p1, q1, p2, q2, hold=True):
        return self._plan(p1, q1, p2, q2).T + config.WRENCH_VERIFY_SEC

    # ----------------------------------------------------------- control
    def _tick(self, plan, tau, rate, last_t):
        """One control update on a fresh nav sample: (nav, t, ControlOutput) or None."""
        ctx = self.ctx
        got = ctx.state.wait_fresh_nav(config.NAV_WAIT_SEC)
        nav = ctx.state.nav
        t = ctx.clock.now()
        if nav is None or not got:
            if last_t is not None and t - last_t > config.NAV_STALE_SEC:
                raise GuidanceFallback('navigation stale')
            return None
        stamp = nav.stamp
        if stamp is None or abs(t - stamp) > 2.0:
            stamp = t - 0.1
        out = self.ctrl.step(t, stamp, nav.pos, nav.vel, nav.quat, nav.angvel,
                             plan.ref(tau), rate)
        self.output.publish(out.force, out.torque)
        return nav, t, out

    def _track_disturbance(self, t):
        f = self.ctrl.disturbance_force()
        self._dist_peak = max(f, self._dist_peak * config.DIST_PEAK_DECAY)
        if t - self._diag_t >= config.DIAG_EVERY:
            self._diag_t = t
            d = self.ctrl.d_hat
            elapsed = self.ctx.timer.elapsed()
            self._diag.append('%.0f:%.0f' % (elapsed, 1000 * f))
            log.info('   diag t=%.1f dist=%.1fmN (%.3f,%.3f,%.3f)m/s2 ff=%.2f sat=%.2f',
                     elapsed, 1000 * f, d[0], d[1], d[2],
                     self.ctrl.ff_scale, self.ctrl.saturation)

    def _hold_until(self, done):
        """Keep station at the current pose until done() or the node stops."""
        plan = hold_plan(self.ctx.pos, self.ctx.quat)
        last = self.ctx.clock.now()
        while self.ctx.alive() and not done():
            tick = self._tick(plan, 0.0, 1.0, last)
            if tick is not None:
                last = tick[1]
                self._track_disturbance(tick[1])

    # ------------------------------------------------------------ lifecycle
    def prepare(self):
        """Hold briefly at the start to measure the disturbance before planning."""
        t0 = self.ctx.clock.now()
        wall_end = time.time() + config.START_OBSERVE_SEC * 10.0
        try:
            self._hold_until(lambda: (self.ctx.clock.now() - t0 >= config.START_OBSERVE_SEC or
                                      time.time() >= wall_end))
        except GuidanceFallback as exc:
            self.close(str(exc))
        log.info('start disturbance %.1fmN', 1000 * self.ctrl.disturbance_force())

    def finish(self):
        """After mission_finished: keep station until the platform stops us."""
        if self.engaged:
            wall_end = time.time() + config.HOLD_AFTER_FINISH_WALL
            try:
                self._hold_until(lambda: not self.engaged or time.time() >= wall_end)
            except GuidanceFallback:
                pass
        self.close('mission complete')

    def close(self, reason):
        """Zero the fans and stop driving them (idempotent)."""
        self.output.close()
        if self.engaged:
            log.warn('wrench guidance closed: %s', reason)
        self.engaged = False

    def stop(self):
        if self.output.used:
            self.close('stop requested')

    def log_trace(self):
        if self._diag:
            log.info('disturbance trace (t:mN): %s', ' '.join(self._diag))

    # ---------------------------------------------------------------- visit
    def visit(self, cp, is_goal=False):
        ctx = self.ctx
        plan = self._plan(ctx.pos, ctx.quat, cp.position, cp.quat)
        t_leg0 = ctx.clock.now()
        log.info('-> CP%d%s  plan %.1fs (d=%.2fm rot=%.0fdeg) dist=%.1fmN  elapsed %.1fs',
                 cp.id, ' (goal)' if is_goal else '', plan.T, plan.d, math.degrees(plan.th),
                 1000 * self.ctrl.disturbance_force(), ctx.timer.elapsed())
        tau, rate, t_prev, t_end = 0.0, 1.0, t_leg0, None
        stable_since = gross_since = None
        last_nav_t = t_leg0
        max_e, min_ff = 0.0, 1.0
        fire_thread = None
        result = {}

        while ctx.alive():
            tick = self._tick(plan, tau, rate, last_nav_t)
            if tick is None:
                continue
            nav, t, out = tick
            last_nav_t = t
            dt = max(0.0, min(0.2, t - t_prev))
            t_prev = t
            e, ea = out.pos_err, out.att_err_deg

            # reference time-warp: wait for the robot if it lags behind
            rate += (self._warp_target(plan, out) - rate) * min(1.0, dt / config.WARP_TAU)
            rate = max(0.0, min(1.0, rate))
            tau += dt * rate
            if tau < plan.T:
                max_e = max(max_e, e)
                min_ff = min(min_ff, self.ctrl.ff_scale)
            elif t_end is None:
                t_end = t
            self._track_disturbance(t)

            # only a gross, persistent failure hands control back to ctl
            if e > config.ABORT_POS_ERR or ea > config.ABORT_ATT_ERR:
                gross_since = t if gross_since is None else gross_since
                if t - gross_since > config.ABORT_PERSIST:
                    raise GuidanceFallback('gross tracking error %.2fm / %.0fdeg' % (e, ea))
            else:
                gross_since = None

            # report running in the background: keep controlling until it returns
            if fire_thread is not None:
                if not fire_thread.is_alive():
                    break
                continue

            if ctx.timer.deadline_hit():
                log.warn('CP%d: deadline reached', cp.id)
                if is_goal:
                    result['lvl'] = ctx.report_checkpoint(
                        cp, judge.metrics(nav.pos, nav.quat, cp), 'deadline')
                return result.get('lvl', 0)

            if t_end is None:
                continue
            why = self._ready_to_report(cp, nav, t, t_end, stable_since)
            if why == 'wait':
                stable_since = None
            elif why == 'stable':
                stable_since = t if stable_since is None else stable_since
            else:
                m = judge.metrics(nav.pos, nav.quat, cp)

                def _fire(m=m, why=why):
                    result['lvl'] = ctx.report_checkpoint(cp, m, why)
                fire_thread = threading.Thread(target=_fire)
                fire_thread.daemon = True
                fire_thread.start()

        took = ctx.clock.now() - t_leg0
        if plan.T > 3.0:
            est = plan.T + config.WRENCH_VERIFY_SEC
            ratio = max(0.9, min(2.0, took / max(est, 1e-3)))
            ctx.time_scale = 0.6 * ctx.time_scale + 0.4 * ratio
        log.info('   CP%d leg %.1fs (plan %.1f) err %.3fm ff>=%.2f dist %.1fmN',
                 cp.id, took, plan.T, max_e, min_ff, 1000 * self.ctrl.disturbance_force())
        ctx.set_pose(cp.position, cp.quat)
        return result.get('lvl', 0)

    @staticmethod
    def _warp_target(plan, out):
        ev = out.err_vec
        if plan.d > 1e-4:
            lag = v_dot(ev, plan.u)
            lat = v_norm(v_sub(ev, v_mul(plan.u, lag)))
        else:
            lag, lat = 0.0, v_norm(ev)
        target = 1.0
        if lag > config.WARP_E0:
            target *= _ramp_down(lag, config.WARP_E0, config.WARP_E1)
        if lat > config.WARP_LAT:
            target *= _ramp_down(lat, config.WARP_LAT, 2.0 * config.WARP_LAT)
        if out.att_err_deg > config.WARP_ATT0:
            target *= _ramp_down(out.att_err_deg, config.WARP_ATT0, config.WARP_ATT1)
        return target

    def _ready_to_report(self, cp, nav, t, t_end, stable_since):
        """'wait', 'stable' (criteria met, keep holding), 'verified' or 'timeout'."""
        over = t - t_end
        if over >= config.WRENCH_FIRE_AFTER:
            return 'timeout'
        m = judge.metrics(nav.pos, nav.quat, cp)
        k = 2.0 if over >= config.WRENCH_LOOSE_AFTER else 1.0
        still = nav.speed < k * config.STABLE_LIN and nav.ang_speed < k * config.STABLE_ANG
        frac = config.TIGHT_FRAC if over < config.WRENCH_RELAX_AFTER else config.RELAX_FRAC
        if not (still and judge.within(m, cp, frac)):
            return 'wait'
        if stable_since is not None and t - stable_since >= config.WRENCH_HOLD_SEC:
            return 'verified'
        return 'stable'
