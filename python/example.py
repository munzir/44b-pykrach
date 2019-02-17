import sys
import pykrach
import time
from math import *

def str(x):
    return ["{0:3.3f}".format(i) for i in x]


print 'creating interface context'
interface_context = pykrach.InterfaceContext('pykrach')
print 'creating world interface'
world = pykrach.WorldInterface(interface_context, 'sim-cmd', 'sim-state')
print 'creating imu interface'
imu = pykrach.FloatingBaseStateSensorInterface(interface_context, 'imu-data')
print 'creating wheels interface'
wheels = pykrach.MotorInterface(interface_context, 'wheels', 'amc-cmd',
                                'amc-state', 2)
print 'creating torso interface'
torso = pykrach.MotorInterface(interface_context, 'torso', 'torso-cmd',
                               'torso-state', 1)
print 'creating left_arm interface'
left_arm = pykrach.MotorInterface(interface_context, 'left-arm', 'llwa-cmd',
                                  'llwa-state', 7)
print 'creating right_arm interface'
right_arm = pykrach.MotorInterface(interface_context, 'right-arm', 'rlwa-cmd',
                                   'rlwa-state', 7)

iter = 0
dt = 0.001
while True:
    try:
        period = 5.0  # sec
        current_command = [
            30 * sin(2 * pi * iter * dt / period),
            30 * sin(2 * pi * iter * dt / period)
        ]
        wheels.CurrentCommand(current_command)

        print 'step'
        success = world.Step()
        if not success:
            sys.exit(0)

        iter = iter + 1

        if iter % 1 == 0:
            imu.UpdateState()
            print 'imu angle: ', "%.4f" % (imu.base_angle * 180.0 / pi)
            print 'imu speed: ', "%.4f" % imu.base_angular_speed

            wheels.UpdateState()
            print 'wheel pos: ', str(wheels.GetPosition())
            print 'wheel vel: ', str(wheels.GetVelocity())
            print 'wheel cur: ', str(wheels.GetCurrent())

            torso.UpdateState()
            print 'torso pos: ', str(torso.GetPosition())
            print 'torso vel: ', str(torso.GetVelocity())
            print 'torso cur: ', str(torso.GetCurrent())

            left_arm.UpdateState()
            print 'L-arm pos: ', str(left_arm.GetPosition())
            print 'L-arm vel: ', str(left_arm.GetVelocity())
            print 'L-arm cur: ', str(left_arm.GetCurrent())

            right_arm.UpdateState()
            print 'R-arm pos: ', str(right_arm.GetPosition())
            print 'R-arm vel: ', str(right_arm.GetVelocity())
            print 'R-arm cur: ', str(right_arm.GetCurrent())

        print 'interface context run'
        interface_context.Run()
    except KeyboardInterrupt:
        sys.exit(0)
