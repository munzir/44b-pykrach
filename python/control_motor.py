import pykrach
import numpy

def str(x):
    return ["{0:3.3f}".format(i) for i in x]

interface_context = pykrach.InterfaceContext('pykrach')

left_arm = pykrach.MotorInterface(interface_context, 'left-arm', 'llwa-cmd', 'llwa-state', 7)

left_arm.UpdateState()

x = numpy.array(left_arm.GetPosition())

print 'Current position: ', str(x)

e = numpy.array([0, 0, 0, 0, 0, 0, 0.2])

x_new = x + e

print 'New position: ', str(x_new)

left_arm.PositionCommand(x_new.tolist())


