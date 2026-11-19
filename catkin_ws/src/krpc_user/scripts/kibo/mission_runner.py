"""Mission orchestration: WHAT to do, independent of HOW the robot flies.

start -> load checkpoints -> pick guidance -> plan route -> visit each
checkpoint within the time budget -> goal -> mission_finished (always, also
on errors, deadline or platform stop).
"""
import threading

from kibo import VERSION
from kibo.core.route import plan_order, route_time
from kibo.guidance.base import GuidanceFallback, MissionContext
from kibo.guidance.ctl_guidance import CtlGuidance
from kibo.guidance.wrench_guidance import WrenchGuidance
from kibo.ros_io import log
from kibo.ros_io.clock import MissionTimer, SimClock, ros_ok
from kibo.ros_io.game_api import GameApi
from kibo.ros_io.state import RobotState


class MissionRunner(object):

    def __init__(self):
        clock = SimClock()
        self.ctx = MissionContext(clock, MissionTimer(clock), RobotState(), GameApi())
        self.wrench = WrenchGuidance(self.ctx)
        self.ctl = CtlGuidance(self.ctx)
        self.active = self.ctl

    # --------------------------------------------------------------- run
    def run(self):
        """Whole mission; called once from the /ib2_user/start callback."""
        ctx = self.ctx
        log.info('kibo %s starting', VERSION)
        try:
            if not self._start():
                return
            mission = ctx.api.load_mission(lambda sec: ctx.clock.sleep(sec, ctx.alive))
            if mission is None:
                log.error('could not load checkpoints; finishing safely')
                return

            self._select(self.wrench if self.wrench.available() else self.ctl)
            ctx.pose_from_nav()
            ctx.goal = mission.goal
            if self.active is self.wrench:
                ctx.time_scale = 1.0
                self.wrench.prepare()

            order = plan_order(ctx.pos, ctx.quat, mission.others, mission.goal,
                               self.active.estimate)
            log.info('plan: %s -> goal %s | model %.0fs | planned at %.1fs',
                     [c.id for c in order], mission.goal_id,
                     route_time(ctx.pos, ctx.quat, order, mission.goal, self.active.estimate),
                     ctx.timer.elapsed())

            for cp in order:
                if not ctx.alive() or ctx.timer.deadline_hit():
                    break
                if not ctx.can_afford(cp):
                    log.warn('skip CP%d: remaining %.0fs not enough', cp.id, ctx.timer.remaining())
                    continue
                self._visit(cp)
            if mission.goal is not None and ctx.alive() and not ctx.timer.deadline_hit():
                self._visit(mission.goal, is_goal=True)

            log.info('report (id, est level): %s | elapsed %.1fs',
                     ctx.reports, ctx.timer.elapsed())
            self.wrench.log_trace()
        except Exception as exc:  # noqa: BLE001 - never die without finishing
            log.error('mission failed: %s', exc)
        finally:
            if ros_ok():
                ctx.api.mission_finished(ctx.timer.elapsed())
            if self.active is self.wrench:
                if ctx.alive():
                    self.wrench.finish()
                else:
                    self.wrench.close('stopped')

    def _start(self):
        ctx = self.ctx
        ctx.api.wait_ready()
        self.ctl.wait_ready()
        t_before = ctx.clock.now()
        if not ctx.api.mission_start():
            return False
        ctx.timer.start(t_before)        # conservative: the judge stamps slightly later
        log.info('clock source: %s, t0=%.2f', ctx.clock.source, t_before)
        return True

    def _select(self, guidance):
        self.active = guidance
        self.ctx.estimator = guidance.estimate

    def _visit(self, cp, is_goal=False):
        """Visit with the active guidance; hand over to ctl if wrench gives up."""
        ctx = self.ctx
        if self.active is self.wrench:
            try:
                return self.wrench.visit(cp, is_goal)
            except GuidanceFallback as exc:
                self.wrench.close(str(exc))
            except Exception as exc:  # noqa: BLE001 - any bug: fans off, use ctl
                self.wrench.close('error: %s' % exc)
            ctx.pose_from_nav()
            self._select(self.ctl)
            if ctx.reported(cp.id):
                return 0
        return self.ctl.visit(cp, is_goal)

    # -------------------------------------------------------------- stop
    def stop(self):
        """Platform stop request (/ib2_user/stop, service thread)."""
        ctx = self.ctx
        ctx.stop_requested = True
        self.wrench.stop()
        self.ctl.stop()
        ctx.api.cancel_pending()
        # report the mission before we are killed, but never block the shutdown
        if ctx.api.started and not ctx.api.finished:
            th = threading.Thread(target=ctx.api.mission_finished,
                                  args=(ctx.timer.elapsed(),))
            th.daemon = True
            th.start()
            th.join(2.0)
