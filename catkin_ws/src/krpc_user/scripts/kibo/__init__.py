"""kibo - Kibo-RPC 7th mission software for Int-Ball2 (team I-ORBIT).

Layers (imports only ever point downwards):

    user_template.py          ROS entry point required by the platform
        mission_runner        WHAT to do: route, time budget, fallback, finish
            guidance/         HOW to fly to one checkpoint (wrench | ctl)
                ros_io/       the only code that touches ROS
                core/         pure Python: maths, models, planning, control
        config                every tunable number, in one place
"""

VERSION = 'v3.5-r1'   # v3.5 behaviour, refactored package layout
