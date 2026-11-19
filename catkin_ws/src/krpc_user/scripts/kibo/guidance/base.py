"""Shared mission context and the Guidance interface."""
from kibo import config
from kibo.core import judge
from kibo.ros_io import log
from kibo.ros_io.clock import ros_ok


class GuidanceFallback(Exception):
    """Raised by a guidance that can no longer fly safely (hand over to ctl)."""


class MissionContext(object):
    """State shared by the runner and all guidance strategies."""

    def __init__(self, clock, timer, state, api):
        self.clock = clock
        self.timer = timer
        self.state = state
        self.api = api
        self.stop_requested = False
        self.pos = list(config.DEFAULT_START_POS)     # last commanded target pose
        self.quat = list(config.DEFAULT_START_QUAT)
        self.goal = None
        self.time_scale = config.INIT_TIME_SCALE      # model -> reality, learned
        self.reports = []                             # [(checkpoint id, est. level)]
        self.estimator = None                         # active guidance .estimate

    # ---------------------------------------------------------------- state
    def alive(self):
        return ros_ok() and not self.stop_requested

    def set_pose(self, pos, quat):
        self.pos, self.quat = list(pos), list(quat)

    def pose_from_nav(self):
        nav = self.state.nav
        if nav is not None:
            self.set_pose(nav.pos, nav.quat)

    def reported(self, cid):
        return any(r[0] == cid for r in self.reports)

    # --------------------------------------------------------------- budget
    def estimate(self, p1, q1, p2, q2, hold=True):
        """Expected leg time [s] with the active guidance (learned scale applied)."""
        return self.estimator(p1, q1, p2, q2, hold) * self.time_scale

    def can_afford(self, cp, extra=0.0):
        """Enough time to visit cp, then reach the goal, before the deadline?"""
        need = self.estimate(self.pos, self.quat, cp.position, cp.quat) + extra
        if self.goal is not None and cp is not self.goal:
            need += self.estimate(cp.position, cp.quat, self.goal.position, self.goal.quat,
                                  hold=False)
            need += config.GOAL_RESERVE_EXTRA_SEC
        return self.timer.remaining() - config.FINISH_GUARD_SEC >= need

    # --------------------------------------------------------------- report
    def report_checkpoint(self, cp, m, why):
        """Log the pose quality, ask the judge (once!) and record the result."""
        if m is None:
            m, lvl = (float('nan'),) * 3, 0
        else:
            lvl = judge.level(m, cp)
        log.info('CP%d %s: d=%.3fm cen=%.1fdeg roll=%.1fdeg -> est Lv%d (t=%.1fs)',
                 cp.id, why, m[0], m[1], m[2], lvl, self.timer.elapsed())
        self.api.checkpoint_passed(cp.id)
        self.reports.append((cp.id, lvl))
        return lvl


class Guidance(object):
    """How to fly to one checkpoint. Subclasses implement estimate() and visit()."""
    name = 'base'

    def __init__(self, ctx):
        self.ctx = ctx

    def estimate(self, p1, q1, p2, q2, hold=True):
        """Model time [s] of one leg (before the learned time scale)."""
        raise NotImplementedError

    def prepare(self):
        """Called once after route-independent setup, before planning."""

    def visit(self, cp, is_goal=False):
        """Fly to cp, verify, report. Returns the estimated level (0 = failed)."""
        raise NotImplementedError

    def finish(self):
        """Called after mission_finished while the node is still alive."""

    def stop(self):
        """Platform stop request (any thread): stop actuating immediately."""
