"""Fallback guidance through the platform controller /ctl/command (v2.1).

Used when ctl is not idle at the start, or after the wrench guidance hands
over. ctl translates, then rotates, then needs 3 s inside 0.1 m / 28 deg
before reporting success; the checkpoint is reported as soon as OUR stricter
judge-based check passes, often during that hold.
"""
from kibo import config
from kibo.core import judge
from kibo.core.route import ctl_leg_time
from kibo.guidance.base import Guidance
from kibo.ros_io import log
from kibo.ros_io import ctl_client
from kibo.ros_io.ctl_client import CtlClient


class CtlGuidance(Guidance):
    name = 'CTL (v2.1)'

    def __init__(self, ctx):
        Guidance.__init__(self, ctx)
        self.ctl = CtlClient()

    def wait_ready(self):
        self.ctl.wait_ready()

    def estimate(self, p1, q1, p2, q2, hold=True):
        return ctl_leg_time(p1, q1, p2, q2, hold)

    def stop(self):
        if self.ctl.active:
            self.ctl.cancel(wait=False)

    def _sample(self, cp):
        """(judge metrics or None, linear speed, angular speed) from the latest nav."""
        nav = self.ctx.state.nav
        if nav is None:
            return None, 0.0, 0.0
        return judge.metrics(nav.pos, nav.quat, cp), nav.speed, nav.ang_speed

    def _learn(self, est, actual):
        ctx = self.ctx
        if est <= 5.0:
            return
        ratio = max(0.8, min(2.5, actual / (est / ctx.time_scale)))
        ctx.time_scale = 0.5 * ctx.time_scale + 0.5 * ratio
        log.info('leg took %.1fs (est %.1fs) -> time scale %.2f', actual, est, ctx.time_scale)

    def visit(self, cp, is_goal=False):
        ctx = self.ctx
        est = ctx.estimate(ctx.pos, ctx.quat, cp.position, cp.quat)
        t_send = ctx.clock.now()
        self.ctl.send(cp.position, cp.quat)
        log.info('-> CP%d%s  est %.1fs  elapsed %.1fs', cp.id, ' (goal)' if is_goal else '',
                 est, ctx.timer.elapsed())

        fired = None
        stable_since = still_since = success_at = None
        corrections = retries = 0
        move_timeout = est * 2.0 + 20.0

        while ctx.alive():
            now = ctx.clock.now()
            m, lin, ang = self._sample(cp)
            still = lin < config.STABLE_LIN and ang < config.STABLE_ANG

            # deadline: report what we have and get out
            if ctx.timer.deadline_hit():
                log.warn('CP%d: deadline reached', cp.id)
                if fired is None and is_goal and m is not None:
                    fired = ctx.report_checkpoint(cp, m, 'deadline')
                if not is_goal:
                    self.ctl.cancel()
                return fired or 0

            # report as soon as our judge-based check passes
            if fired is None and m is not None:
                tight = (success_at is None or
                         now - success_at < config.CTL_POST_SUCCESS_TIGHT_SEC)
                frac = config.TIGHT_FRAC if tight else config.RELAX_FRAC
                if still and judge.within(m, cp, frac):
                    if stable_since is None:
                        stable_since = now
                    elif now - stable_since >= config.CTL_STABLE_HOLD_SEC:
                        fired = ctx.report_checkpoint(cp, m, 'verified')
                        if is_goal:
                            return fired
                else:
                    stable_since = None

            # action progress
            res = self.ctl.poll()
            if res is not None and success_at is None:
                if res == ctl_client.SUCCESS:
                    success_at = now
                    self._learn(est, now - t_send)
                    ctx.set_pose(cp.position, cp.quat)
                    if fired is not None:
                        return fired
                else:
                    log.warn('CP%d: move ended with type=%s', cp.id, res)
                    if retries < config.CTL_MOVE_RETRIES and ctx.can_afford(cp):
                        retries += 1
                        ctx.pose_from_nav()
                        self.ctl.send(cp.position, cp.quat)
                        t_send, stable_since = ctx.clock.now(), None
                        continue
                    success_at = now            # fall through to "report anyway"

            if fired is not None and success_at is not None:
                return fired

            # after success: one correction move if precision is poor, else report
            if success_at is not None and fired is None:
                if now - success_at >= config.CTL_POST_SUCCESS_RELAX_SEC:
                    lvl = judge.level(m, cp) if m else 0
                    if (lvl < 3 and corrections < config.CTL_MAX_CORRECTIONS and
                            ctx.can_afford(cp, extra=config.CTL_HOLD_SEC + 4.0)):
                        corrections += 1
                        log.warn('CP%d: precision Lv%d (d=%.3f c=%.1f r=%.1f), correcting',
                                 cp.id, lvl, m[0], m[1], m[2])
                        self.ctl.send(cp.position, cp.quat)
                        t_send, success_at, stable_since = ctx.clock.now(), None, None
                        est = config.CTL_HOLD_SEC + 4.0
                        continue
                    return ctx.report_checkpoint(cp, m, 'fallback')

            # robot stopped near the target but ctl never confirms (disturbance)
            if still:
                still_since = now if still_since is None else still_since
            else:
                still_since = None
            if (success_at is None and still_since is not None and
                    now - still_since >= config.CTL_STALL_STILL_SEC and
                    now - t_send > est + config.CTL_STALL_GRACE_SEC and
                    m is not None and judge.level(m, cp) >= 1):
                log.warn('CP%d: ctl did not confirm, robot still -> evaluate', cp.id)
                success_at = now - config.CTL_POST_SUCCESS_TIGHT_SEC
                ctx.set_pose(cp.position, cp.quat)

            # stuck move
            if success_at is None and now - t_send > move_timeout:
                log.warn('CP%d: move timeout (%.1fs)', cp.id, now - t_send)
                self.ctl.cancel()
                success_at = ctx.clock.now() - config.CTL_POST_SUCCESS_RELAX_SEC
                corrections = config.CTL_MAX_CORRECTIONS

            ctx.clock.sleep(config.CTL_POLL_SEC, ctx.alive)
        return fired or 0
