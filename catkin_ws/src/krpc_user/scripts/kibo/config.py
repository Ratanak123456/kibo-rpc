"""Every tunable number of the mission software, grouped by concern.

Units are SI unless stated (m, s, rad, N). Times are SIMULATION time: the
judge scores in sim time and the web simulator runs slower than real time.
"""

# --------------------------------------------------------------------- mission
TIME_LIMIT_SEC = 300.0          # judged mission time limit
FINISH_GUARD_SEC = 6.0          # always report mission_finished before limit - this
GOAL_RESERVE_EXTRA_SEC = 6.0    # slack kept for the goal when deciding to visit a CP
WAIT_SERVICE_TIMEOUT = 20.0     # [s wall] waiting for platform services at start
CHECKPOINT_CALL_TIMEOUT = 15.0  # [s wall] checkpoint_passed action timeout
LOAD_RETRIES = 3                # get_checkpoints attempts

DEFAULT_START_POS = (0.200, 0.0, 0.0)       # dock exit, used if nav is missing
DEFAULT_START_QUAT = (0.0, 0.0, 0.0, 1.0)
DEFAULT_POS_BANDS = [(0.1, 3), (0.2, 2), (0.3, 1)]       # (radius m, level)
DEFAULT_ANG_BANDS = [(10.0, 3), (20.0, 2), (30.0, 1)]    # (angle deg, level)

# ------------------------------------------------- checkpoint verification
TIGHT_FRAC = 0.55               # report when all metrics < 55 % of the Lv3 margin
RELAX_FRAC = 0.85               # after waiting long enough accept < 85 %
STABLE_LIN = 0.012              # [m/s]   "robot is still" thresholds
STABLE_ANG = 0.035              # [rad/s]

# ------------------------------------------- guidance selection / safety
USE_WRENCH = True               # False = always fly with the platform ctl (v2.1)
CTL_STAND_BY = 0                # CtlStatusType.STAND_BY: ctl idle, fans are ours
MODE_DECISION_WAIT = 5.0        # [s wall] wait for ctl status + nav at start

# --------------------------------------------------- wrench guidance (v3)
WRENCH_EFF = 0.75               # share of fan capacity used by planned motion
STRONG_DIST_N = 0.045           # [N] disturbance peak above which ...
STRONG_DIST_EFF = 0.60          # ... legs are planned with this capacity share
DIST_PEAK_DECAY = 0.999         # per control tick decay of the disturbance peak
WRENCH_VERIFY_SEC = 1.0         # planning overhead per leg (settle + verify)
WRENCH_HOLD_SEC = 0.4           # stable time before reporting a checkpoint
WRENCH_RELAX_AFTER = 4.0        # [s] after plan end -> RELAX_FRAC criteria
WRENCH_LOOSE_AFTER = 6.0        # [s] after plan end -> 2x stillness limits
WRENCH_FIRE_AFTER = 20.0        # [s] after plan end -> report anyway
START_OBSERVE_SEC = 1.5         # hold at start to measure the disturbance
NAV_WAIT_SEC = 0.25             # [s wall] wait for a fresh nav sample per tick
NAV_STALE_SEC = 1.0             # [s] no nav for this long -> hand back to ctl
HOLD_AFTER_FINISH_WALL = 600.0  # [s wall] max station keeping after the mission

# reference time-warp: the reference waits for the robot instead of running away
WARP_E0, WARP_E1 = 0.05, 0.18   # [m] lag behind reference: slow from E0, pause at E1
WARP_LAT = 0.15                 # [m] lateral error that also slows the reference
WARP_ATT0, WARP_ATT1 = 20.0, 40.0   # [deg] attitude error: slow .. pause
WARP_TAU = 0.4                  # [s] rate smoothing time constant

# gross failure -> stop driving the fans, hand control back to ctl
ABORT_POS_ERR = 1.0             # [m]
ABORT_ATT_ERR = 60.0            # [deg]
ABORT_PERSIST = 3.0             # [s]

DIAG_EVERY = 3.0                # [s] disturbance diagnostics in the log

# ------------------------------------------- leg trajectory (core.trajectory)
TRAJ_V_MAX = 0.35               # [m/s] cruise speed cap
TRAJ_W_MAX = 0.8                # [rad/s] rotation rate cap
TRAJ_LAMBDAS = [0.25 + 0.05 * i for i in range(15)]   # force/torque split search

# -------------------------------------------- wrench controller (core.controller)
CTRL_KP, CTRL_KD, CTRL_KI = 1.1, 1.75, 0.12   # position PID [1/s^2, 1/s, 1/s^3]
CTRL_KPA, CTRL_KDA = 3.2, 3.6                 # attitude PD
CTRL_A_FB_MAX = 0.045           # [m/s^2] PD authority
CTRL_I_LIM = 0.004              # [m/s^2] integral authority
CTRL_I_ZONE = 0.10              # [m] integrate only when closer than this
CTRL_FAN_LIMIT_FRAC = 0.92      # never command a fan above 92 % of its maximum
CTRL_PRED_MAX = 0.25            # [s] max nav-delay prediction
CTRL_SAT_BISECT = 10            # bisection steps for priority saturation

# ------------------------------------- disturbance observer (core.observer)
OBS_WIN = 1.0                   # [s] velocity-change window
OBS_GAIN = 0.25                 # low-pass gain per update
OBS_D_LIM = 0.045               # [m/s^2] max compensated disturbance

# ------------------------------- platform ctl time model (core.route, v2.1)
CTL_V_MAX = 0.20                # pos_profile/v_max
CTL_A_MAX = 0.056               # f_max / mass (nominal)
CTL_W_MAX = 0.4363              # att_profile/w_max
CTL_ALPHA_MAX = 0.60            # t_max / I (nominal)
CTL_HOLD_SEC = 3.0              # ctl/duration_goal
CTL_LEG_OVERHEAD_SEC = 0.8      # action round trip, nav latency
INIT_TIME_SCALE = 1.20          # model -> reality factor, learned online

# ----------------------------------------- ctl guidance (guidance.ctl_guidance)
CTL_STABLE_HOLD_SEC = 0.6       # judge's pose sample lags ~0.2 s
CTL_POST_SUCCESS_TIGHT_SEC = 3.0
CTL_POST_SUCCESS_RELAX_SEC = 5.5
CTL_STALL_STILL_SEC = 2.0       # still this long without ctl success ...
CTL_STALL_GRACE_SEC = 8.0       # ... and this far past the estimate -> evaluate
CTL_MAX_CORRECTIONS = 1
CTL_MOVE_RETRIES = 1
CTL_POLL_SEC = 0.05
