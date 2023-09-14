from walking.foot_step_planner import *


if __name__ == '__main__':
    # planner = foot_step_planner(0.06, 0.04, 0.1, 0.34, 0.044)
    planner = foot_step_planner(0.05, 0.03, 0.2, 0.3, 0.08)
    foot_step = planner.calculate(0.5, 0.0, 0, 0, 0, 0, 'right', 'start')
    # foot_step = planner.calculate(1.0, 0.0, 0.5, 0.5, 0.0, 0.1, 'right', 'start')
    for i in foot_step:
        print(i)

# foot_step:
# [0.0, 0, 0, 0, 'both']
# [0.6, 0, -0.08, 0, 'right']
# [0.9, 0.05, 0.08, 0.0, 'left']
# [1.2, 0.1, -0.08, 0.0, 'right']
# [1.5, 0.15, 0.08, 0.0, 'left']
# [1.8, 0.2, -0.08, 0.0, 'right']
# [2.1, 0.25, 0.08, 0.0, 'left']
# [2.4, 0.3, -0.08, 0.0, 'right']
# [2.7, 0.35, 0.08, 0.0, 'left']
# [3.0, 0.4, -0.08, 0.0, 'right']
# [3.3, 0.45, 0.08, 0.0, 'left']
# [3.6, 0.5, -0.08, 0.0, 'right']
# [3.9, 0.5, 0.08, 0, 'left']
# [4.2, 0.5, 0.0, 0, 'both']
# [6.2, 0.5, 0.0, 0, 'both']