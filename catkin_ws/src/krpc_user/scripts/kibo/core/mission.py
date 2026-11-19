"""Mission data: checkpoints as returned by /mission/get_checkpoints."""
import json

from kibo import config
from kibo.core.geometry import q_look_at, q_normalize


class Checkpoint(object):
    """One checkpoint: where to be, how to be oriented, and its scoring bands.

    position  [x, y, z] target position (iss_body / dock frame)
    quat      [x, y, z, w] target attitude (camera +X looks at the marker)
    marker    [x, y, z] AR marker position (used by the judge for centering)
    *_bands   [(threshold, level), ...] sorted by threshold, e.g. (0.1, 3)
    """
    __slots__ = ('id', 'position', 'quat', 'marker', 'pos_bands', 'cen_bands', 'roll_bands')

    def __init__(self, cid, position, quat, marker, pos_bands, cen_bands, roll_bands):
        self.id = int(cid)
        self.position = [float(v) for v in position]
        self.quat = q_normalize([float(v) for v in quat])
        self.marker = [float(v) for v in marker]
        self.pos_bands = pos_bands
        self.cen_bands = cen_bands
        self.roll_bands = roll_bands

    def __repr__(self):
        return 'CP%d' % self.id


class Mission(object):
    """All checkpoints of a run, with the goal split out."""

    def __init__(self, checkpoints, goal_id):
        self.checkpoints = checkpoints
        self.goal_id = goal_id
        self.goal = next((c for c in checkpoints if c.id == goal_id), None)
        self.others = [c for c in checkpoints if c.id != goal_id]


def parse_mission(message):
    """Parse the JSON string of the /mission/get_checkpoints response."""
    data = json.loads(message)
    raw = data.get('checkpoints', data.get('Checkpoints', []))
    cps = []
    for c in raw:
        cid = c.get('checkpoint_id', c.get('checkpointid', c.get('id')))
        pos = c['position']
        att = c.get('attitude')
        att = att if isinstance(att, dict) else {}
        marker = (c.get('marker_pose') or {}).get('position') or pos
        quat = att.get('quaternion') or q_look_at(pos, marker)
        cps.append(Checkpoint(
            cid, pos, quat, marker,
            _bands(c.get('score_bands'), 'radius', config.DEFAULT_POS_BANDS),
            _bands(att.get('centering_bands'), 'angle', config.DEFAULT_ANG_BANDS),
            _bands(att.get('roll_bands'), 'angle', config.DEFAULT_ANG_BANDS)))
    goal_id = data.get('goalpoint_id')
    return Mission(cps, int(goal_id) if goal_id is not None else None)


def _bands(raw, key, default):
    out = []
    if isinstance(raw, list):
        for b in raw:
            try:
                thr = b.get(key, b.get('margin', b.get('threshold', b.get('distance'))))
                out.append((float(thr), int(b.get('level', b.get('score')))))
            except Exception:  # noqa: BLE001 - skip malformed band, keep the rest
                continue
    out.sort()
    return out if out else list(default)
