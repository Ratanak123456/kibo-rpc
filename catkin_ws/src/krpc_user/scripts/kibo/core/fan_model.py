"""Int-Ball2 rigid body and fan model, ported from the platform software.

/ctl/wrench -> fsm -> fan_alloc() (thrust_allocator.cpp) -> 8 fan forces.
If 4+ fans would exceed their maximum, the fsm switches to a crude
"top-4 fans full, rest off" mode, so every command is kept below the limit.

Capacity is strongly direction dependent: ~0.18 N along body +-X but only
~0.07 N in the weakest diagonal, and very little torque.
"""
from kibo.core.geometry import v_mul, v_norm

IB2_MASS = 3.216                                       # [kg]
IB2_INERTIA = ((0.0136245, 0.0000329, -0.0000655),     # [kg m^2] body frame
               (0.0000329, 0.0139967, 0.0000002),
               (-0.0000655, 0.0000002, 0.0130568))
IB2_INERTIA_DIAG_INV = ((1 / 0.01362, 0, 0), (0, 1 / 0.0140, 0), (0, 0, 1 / 0.01306))
FAN_FMAX = 0.06                                        # [N] per fan

# allocation matrices (ctl_only/ctl.yaml fan/Wp, fan/Wm): rows = fans,
# columns = (Fx, Fy, Fz, Tx, Ty, Tz) for positive / negative demands
_WP = ((0, 0, 0, 0, 0, 7.3257), (0, 0.6027, 0, 19.8198, 0, 0),
       (0, 0.6027, 0.4911, 0, 13.1935, 0), (0, 0, 0.4911, 19.8198, 13.1935, 7.3257),
       (0.3315, 0, 0.4911, 19.8198, 0, 0), (0.3315, 0, 0, 0, 13.1935, 0),
       (0.3315, 0.6027, 0, 19.8198, 13.1935, 7.3257), (0.3315, 0.6027, 0.4911, 0, 0, 7.3257))
_WM = ((0.3315, 0.6027, 0.4911, 19.8198, 13.1935, 0), (0.3315, 0, 0.4911, 0, 13.1935, 7.3257),
       (0.3315, 0, 0, 19.8198, 0, 7.3257), (0.3315, 0.6027, 0, 0, 0, 0),
       (0, 0.6027, 0, 0, 13.1935, 7.3257), (0, 0.6027, 0.4911, 19.8198, 0, 7.3257),
       (0, 0, 0.4911, 0, 0, 0), (0, 0, 0, 19.8198, 13.1935, 0))
_ZERO = (0.0, 0.0, 0.0)


def fan_alloc(F, T):
    """Fan forces [N] the fsm commands for body force F [N] and torque T [Nm]."""
    y = (F[0], F[1], F[2], T[0], T[1], T[2])
    f = []
    for i in range(8):
        s = 0.0
        wp, wm = _WP[i], _WM[i]
        for k in range(6):
            yk = y[k]
            s += wp[k] * yk if yk > 0.0 else -wm[k] * yk
        f.append(s)
    mn = min(f)
    return [x - mn for x in f]


def max_fan(F, T):
    """Largest single fan force needed for (F, T)."""
    return max(fan_alloc(F, T))


def cap_force(dir_body):
    """Max force [N] along a body direction with every fan within its limit."""
    n = v_norm(dir_body)
    if n < 1e-9:
        return 1.0
    return FAN_FMAX / max(max_fan(v_mul(dir_body, 1.0 / n), _ZERO), 1e-9)


def cap_torque(axis_body):
    """Max torque [Nm] about a body axis with every fan within its limit."""
    n = v_norm(axis_body)
    if n < 1e-9:
        return 1.0
    return FAN_FMAX / max(max_fan(_ZERO, v_mul(axis_body, 1.0 / n)), 1e-9)
