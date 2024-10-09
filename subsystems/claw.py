from pybricks.ev3devices import Motor
from pybricks.messaging import NumericMailbox
from pybricks.parameters import Stop

from constants import (CLAW_COLOR_SENSOR_MAILBOX,
                       CLAW_MOTOR_PORT)
from utils.utils_server import int_to_color_claw


class Claw:
    _claw_motor = Motor(CLAW_MOTOR_PORT)

    def __init__(self, server):
        self._claw_color_sensor_mbox = NumericMailbox(CLAW_COLOR_SENSOR_MAILBOX, server)
        self._claw_motor.reset_angle(0)
        self._claw_motor.hold()

    def grab(self):
        angle = self._claw_motor.run_until_stalled(-200, Stop.HOLD, 97)
        return angle > -280

    def open(self):
        self._claw_motor.run_target(200, 0, Stop.COAST, True)

    def hold(self):
        self._claw_motor.hold()

    def claw_sensor_color(self):
        return int_to_color_claw(int(self._claw_color_sensor_mbox.read()))
