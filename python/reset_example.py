from pykrach import *
if_cx = InterfaceContext('reset_example-py')
world = WorldInterface(if_cx, 'sim-cmd', 'sim-state')
reset_pose = {
    'heading': 0.0,
    'q_base': -1.74,
    'xyz': (0.0, 0.0, 0.27),
    'q_lwheel': 0.0,
    'q_rwheel': 0.0,
    'q_waist': 2.71,
    'q_torso': 0.0,
    'q_left_arm': (1.130, -1.000, 0.000, -1.570, -0.000, 1.000, -1.104),
    'q_right_arm': (-1.130, 1.000, -0.000, 1.570, 0.000, -1.000, -0.958),
    'q_camera': (0.0, 0.0),
    'init_with_balance_pose': True
}

world.Reset(reset_pose)

