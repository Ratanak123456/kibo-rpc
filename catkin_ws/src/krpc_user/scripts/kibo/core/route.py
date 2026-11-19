"""Route planning: visiting order that minimises total predicted flight time.

The cost of a leg comes from the active guidance (cost(p1, q1, p2, q2, hold)),
so rotation and fan-capacity effects are included, not just distance.
Held-Karp dynamic programming is exact for up to 13 checkpoints (the game has
at most 11 plus the goal, which is always visited last).
"""
import math

from kibo import config
from kibo.core.geometry import dist, q_angle

HELD_KARP_MAX = 13


def _bangbang(x, vmax, amax):
    if x <= 1e-6:
        return 0.0
    if x > vmax * vmax / amax:
        return x / vmax + vmax / amax
    return 2.0 * math.sqrt(x / amax)


def ctl_leg_time(p1, q1, p2, q2, hold=True):
    """Model time [s] of one platform-ctl MOVE_TO_ABSOLUTE (translate, then rotate)."""
    t = _bangbang(dist(p1, p2), config.CTL_V_MAX, config.CTL_A_MAX)
    t += _bangbang(q_angle(q1, q2), config.CTL_W_MAX, config.CTL_ALPHA_MAX)
    return t + config.CTL_LEG_OVERHEAD_SEC + (config.CTL_HOLD_SEC if hold else 1.0)


def plan_order(start_pos, start_q, cps, goal, cost):
    """Order of cps (Checkpoint list) from the start pose; goal (or None) is last."""
    n = len(cps)
    if n == 0:
        return []
    d_start = [cost(start_pos, start_q, c.position, c.quat) for c in cps]
    d = [[cost(a.position, a.quat, b.position, b.quat) for b in cps] for a in cps]
    if goal is not None:
        d_goal = [cost(c.position, c.quat, goal.position, goal.quat, hold=False) for c in cps]
    else:
        d_goal = [0.0] * n
    if n > HELD_KARP_MAX:
        idx = _two_opt(_nearest_neighbour(d_start, d), d_start, d, d_goal)
    else:
        idx = _held_karp(d_start, d, d_goal)
    return [cps[i] for i in idx]


def route_time(start_pos, start_q, order, goal, cost):
    """Predicted total time of a route (for logging / budgeting)."""
    total, p, q = 0.0, start_pos, start_q
    for c in order:
        total += cost(p, q, c.position, c.quat)
        p, q = c.position, c.quat
    if goal is not None:
        total += cost(p, q, goal.position, goal.quat, hold=False)
    return total


def _held_karp(d_start, d, d_goal):
    n = len(d_start)
    inf = float('inf')
    full = (1 << n) - 1
    dp = [[inf] * n for _ in range(full + 1)]
    par = [[-1] * n for _ in range(full + 1)]
    for j in range(n):
        dp[1 << j][j] = d_start[j]
    for mask in range(1, full + 1):
        row = dp[mask]
        for j in range(n):
            cur = row[j]
            if cur == inf:
                continue
            dj = d[j]
            for k in range(n):
                if mask & (1 << k):
                    continue
                nm = mask | (1 << k)
                val = cur + dj[k]
                if val < dp[nm][k]:
                    dp[nm][k] = val
                    par[nm][k] = j
    last = min(range(n), key=lambda j: dp[full][j] + d_goal[j])
    seq, mask, j = [], full, last
    while j != -1:
        seq.append(j)
        pj = par[mask][j]
        mask ^= (1 << j)
        j = pj
    seq.reverse()
    return seq


def _nearest_neighbour(d_start, d):
    order, cur, left = [], None, list(range(len(d_start)))
    while left:
        nxt = min(left, key=lambda k: d_start[k] if cur is None else d[cur][k])
        left.remove(nxt)
        order.append(nxt)
        cur = nxt
    return order


def _two_opt(order, d_start, d, d_goal):
    def total(o):
        c = d_start[o[0]] + d_goal[o[-1]]
        for a, b in zip(o, o[1:]):
            c += d[a][b]
        return c
    best, improved = list(order), True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for k in range(i + 1, len(best)):
                cand = best[:i] + best[i:k + 1][::-1] + best[k + 1:]
                if total(cand) < total(best) - 1e-9:
                    best, improved = cand, True
    return best
