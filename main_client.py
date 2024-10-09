#!/usr/bin/env pybricks-micropython

from pybricks.messaging import BluetoothMailboxClient, NumericMailbox
from pybricks.ev3devices import UltrasonicSensor, ColorSensor
from utils.utils_client import rgb_to_color_claw, color_to_int_claw
from pybricks.tools import wait
from constants import (
    SERVER_NAME,
    CLAW_COLOR_SENSOR_PORT,
    OBSTACLE_SENSOR_PORT,
    TOP_OBJECT_SENSOR_PORT,
    CLAW_COLOR_SENSOR_MAILBOX,
    OBSTACLE_SENSOR_MAILBOX,
    TOP_OBJECT_SENSOR_MAILBOX
    #    BOTTOM_OBJECT_SENSOR_MAILBOX
)

# SENSORES
obstacle_sensor = UltrasonicSensor(OBSTACLE_SENSOR_PORT)
top_object_sensor = UltrasonicSensor(TOP_OBJECT_SENSOR_PORT)
claw_color_sensor = ColorSensor(CLAW_COLOR_SENSOR_PORT)

# CONEXÃO
conn = BluetoothMailboxClient()

# MAILBOXES
obstacle_sensor_mailbox = NumericMailbox(OBSTACLE_SENSOR_MAILBOX, conn)
top_object_sensor_mailbox = NumericMailbox(TOP_OBJECT_SENSOR_MAILBOX, conn)
claw_color_sensor_mailbox = NumericMailbox(CLAW_COLOR_SENSOR_MAILBOX, conn)

print('establishing connection...')
conn.connect(SERVER_NAME)
print('connected!')

while True:
    # obstacle_sensor_mailbox.send(600)
    obstacle_sensor_mailbox.send(obstacle_sensor.distance())
    top_object_sensor_mailbox.send(top_object_sensor.distance())
    # top_object_sensor_mailbox.send(150)
    claw_color_sensor_mailbox.send(color_to_int_claw(rgb_to_color_claw(claw_color_sensor.rgb())))
    wait(150)
