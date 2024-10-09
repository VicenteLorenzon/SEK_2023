from constants import (LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, SENSOR_1_PORT, SENSOR_3_PORT,
                       GYRO_PORT, TURNING_D, TURNING_I, TURNING_P, BOTTOM_OBJECT_SENSOR_PORT, STRAIGHT_P, STRAIGHT_I,
                       STRAIGHT_D)
from pybricks.ev3devices import (Motor, ColorSensor, GyroSensor, UltrasonicSensor)
from pybricks.messaging import NumericMailbox
from pybricks.robotics import DriveBase
from constants import (OBSTACLE_SENSOR_MAILBOX, TOP_OBJECT_SENSOR_MAILBOX,
                       WHEEL_DIAMETER, AXLE_TRACK)
from utils import utils_server
from utils.pid import PID
from pybricks.parameters import Color, Direction
from pybricks.tools import StopWatch


class Side:
    FRONT = 0
    BACK = 180
    LEFT = 270
    RIGHT = 90


class DrivePosition:
    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading


class Drive:
    _left_motor = Motor(LEFT_MOTOR_PORT, positive_direction=Direction.COUNTERCLOCKWISE)
    _right_motor = Motor(RIGHT_MOTOR_PORT, positive_direction=Direction.COUNTERCLOCKWISE)

    _drive_base = DriveBase(_left_motor, _right_motor, WHEEL_DIAMETER, AXLE_TRACK)

    _gyro = GyroSensor(GYRO_PORT)
    _left_sensor = ColorSensor(SENSOR_1_PORT)
    _right_sensor = ColorSensor(SENSOR_3_PORT)
    _bottom_object_sensor = UltrasonicSensor(BOTTOM_OBJECT_SENSOR_PORT)

    _position = DrivePosition(0, 0, Side.LEFT)
    _prohibited_squares = [(0, 1), (2, 1), (4, 1), (0, 3), (2, 3), (4, 3)]

    def __init__(self, server):
        self._obstacle_sensor_mbox = NumericMailbox(OBSTACLE_SENSOR_MAILBOX, server)
        self._top_object_sensor_mbox = NumericMailbox(TOP_OBJECT_SENSOR_MAILBOX, server)
        self._drive_base.stop()
        self._gyro.reset_angle(0)

    def set_motor_speeds(self, left=None, right=None):
        if left is not None:
            self._left_motor.run(left)
        if right is not None:
            self._right_motor.run(right)

    def turn_angle(self, angle: int):
        setpoint = self.get_gyro_heading() + angle

        # pid = PID(TURNING_P, TURNING_I, TURNING_D, setpoint)

        #watch = StopWatch()
        #watch.resume()
        #while self.get_gyro_heading() != setpoint:
        #    if watch.time() >= 3000:
        #        break
        #    turn_rate = pid(self.get_gyro_heading())
        #    self._drive_base.drive(0, turn_rate)
        #watch.pause()

        while abs(self.get_gyro_heading() - setpoint) > 1:
            degrees_left = setpoint - self.get_gyro_heading()

            if degrees_left < 0:
                multiplier = -1
            else:
                multiplier = 1

            if abs(degrees_left) > 50:
                speed = 150 * multiplier
            else:
                speed = max(abs(degrees_left) * 3 * multiplier, (20 * multiplier))

            self._drive_base.drive(0, speed)

        self._drive_base.stop()
        self.hold()

    def turn_to_side(self, side):
        current_heading = self.get_gyro_heading() % 360
        current_heading = current_heading if current_heading >= 0 else current_heading + 360
        angle = int(side) - current_heading
        if abs(angle) > 180:
            angle = angle + 360 if angle < 0 else angle - 360

        self.turn_angle(angle)
        self._position.heading = side

    def drive_straight(self, speed, distance=None, stop_conditions=None, explicit_direction=None):
        if stop_conditions is None:
            stop_conditions = []

        if distance is not None:
            speed = abs(speed) if distance > 0 else -abs(speed)

        if explicit_direction is None:
            direction = self.get_gyro_heading()
        else:
            current_direction = self.get_gyro_heading() % 360
            current_direction = current_direction if current_direction >= 0 else current_direction + 360
            angle = explicit_direction-current_direction
            if abs(angle) > 180:
                angle = angle + 360 if angle < 0 else angle - 360
            direction = self.get_gyro_heading() + angle

        pid_straight = PID(TURNING_P, TURNING_I, TURNING_D, direction)
        if distance is not None:
            if speed > 0:
                pid_speed = PID(STRAIGHT_P, STRAIGHT_I, STRAIGHT_D, distance, output_limits=(45, speed))
            else:
                pid_speed = PID(STRAIGHT_P, STRAIGHT_I, STRAIGHT_D, distance, output_limits=(speed, -45))

        self._drive_base.reset()
        while True:
            for condition in stop_conditions:
                if condition():
                    self._drive_base.stop()
                    self.hold()
                    return
            if distance is not None:
                distance_traveled = self._drive_base.distance()
                if abs(distance_traveled) >= abs(distance): # abs(distance_traveled) > (abs(distance)-5):
                    self._drive_base.stop()
                    self.hold()
                    return
                drive_speed = pid_speed(distance_traveled)
            else:
                drive_speed = speed

            turn_rate = pid_straight(self.get_gyro_heading())
            self._drive_base.drive(drive_speed, turn_rate)

    def drive_straight_squares(self, squares: int, explicit_direction=None):
        self.drive_straight(250, distance=(squares*300), explicit_direction=explicit_direction)

        if self.get_estimated_heading() == Side.FRONT:
            self._position.y -= squares
        elif self.get_estimated_heading() == Side.BACK:
            self._position.y += squares
        elif self.get_estimated_heading() == Side.LEFT:
            self._position.x -= squares
        elif self.get_estimated_heading() == Side.RIGHT:
            self._position.x += squares

    def align_on_color(self, colors, limit_angle=None):
        sensor_readings = []
        right_backwards = False
        left_backwards = False
        left_done = False
        right_done = False
        initial_angle = self.get_gyro_heading()

        while not (left_done and right_done):
            sensor_readings.append(self.left_sensor_color())
            sensor_readings.append(self.right_sensor_color())

            if self.right_sensor_color() in colors:
                self.set_motor_speeds(right=-75)
                right_backwards = True

            if not right_backwards:
                self.set_motor_speeds(right=100)

            if self.left_sensor_color() in colors:
                self.set_motor_speeds(left=-75)
                left_backwards = True

            if not left_backwards:
                self.set_motor_speeds(left=100)

            if left_backwards and (self.left_sensor_color() == Color.WHITE):
                self.stop(right=False)

            if right_backwards and (self.right_sensor_color() == Color.WHITE):
                self.stop(left=False)

            if (right_backwards and left_backwards and
                    (self.right_sensor_color() == Color.WHITE) and (self.left_sensor_color() == Color.WHITE)):
                self.stop()
                break

            if (limit_angle is not None) and (abs(initial_angle - self.get_gyro_heading()) > limit_angle):
                self.stop()
                break

        return len([i for i in sensor_readings if i == Color.YELLOW]) > 2

    def stop(self, left=True, right=True):
        if left:
            self._left_motor.stop()
        if right:
            self._right_motor.stop()

    def hold(self, left=True, right=True):
        if left:
            self._left_motor.hold()
        if right:
            self._right_motor.hold()

    def left_motor_angle(self):
        return self._left_motor.angle()

    def right_motor_angle(self):
        return self._right_motor.angle()

    def any_sensor_sees_color(self, colors=None):
        if colors is None:
            return (
                    (self.left_sensor_color() != Color.WHITE) or
                    (self.right_sensor_color() != Color.WHITE)
            )
        else:
            return (
                    (self.left_sensor_color() in colors) or
                    (self.right_sensor_color() in colors)
            )

    def left_sensor_color(self):
        return utils_server.rgb_to_color(self._left_sensor.rgb())

    def right_sensor_color(self):
        return utils_server.rgb_to_color(self._right_sensor.rgb())

    def get_position_x(self):
        return self._position.x

    def get_position_y(self):
        return self._position.y

    def get_estimated_heading(self):
        return self._position.heading

    def set_position_x(self, x):
        self._position.x = x

    def set_position_y(self, y):
        self._position.y = y

    def set_heading(self, heading):
        self._position.heading = heading

    def measure_tube(self):
        top_readings = []
        bottom_readings = []
        self._drive_base.reset()
        self._drive_base.drive(-25, 0)
        while True:
            bottom_reading = self.bottom_object_sensor_distance()
            top_reading = self._top_object_sensor_mbox.read()

            if top_reading is not None:
                top_readings.append(top_reading)
            if bottom_reading is not None:
                bottom_readings.append(bottom_reading)

            if self._drive_base.distance() <= -50:
                self._drive_base.stop()
                self.hold()
                break

        bottom_readings = [i for i in bottom_readings if i < 130]
        tube_distance = sum(bottom_readings) / len(bottom_readings)
        is_adult = len([i for i in top_readings if i <= 130]) > len([i for i in top_readings if i > 130])

        return tube_distance, is_adult

    def is_seeing_obstacle_far(self):
        read = self._obstacle_sensor_mbox.read()
        if read is None:
            return False
        else:
            return read < 450

    def is_seeing_obstacle_very_close(self):
        read = self._obstacle_sensor_mbox.read()
        if read is None:
            return False
        else:
            return read < 85

    def bottom_object_sensor_distance(self):
        return self._bottom_object_sensor.distance()

    def is_seeing_person(self):
        return self.bottom_object_sensor_distance() < 130

    def is_seeing_adult(self):
        read = self._top_object_sensor_mbox.read()
        if read is None:
            return False
        else:
            return read < 200

    def add_obstacle(self, position):
        self._prohibited_squares.append(position)

    def get_prohibited_squares(self):
        return self._prohibited_squares

    def get_side_of_adjacent_square(self, square):
        x, y = square
        if (x > self._position.x) and (y == self._position.y):
            return Side.RIGHT
        elif (x < self.get_position_x()) and (y == self.get_position_y()):
            return Side.LEFT
        elif (y > self.get_position_y()) and (x == self.get_position_x()):
            return Side.BACK
        elif (y < self.get_position_y()) and (x == self.get_position_x()):
            return Side.FRONT
        else:
            return None

    def get_square_ahead(self):
        x = self.get_position_x()
        y = self.get_position_y()

        if self.get_estimated_heading() == Side.FRONT:
            return tuple((x, y - 1))
        elif self.get_estimated_heading() == Side.BACK:
            return tuple((x, y + 1))
        elif self.get_estimated_heading() == Side.LEFT:
            return tuple((x - 1, y))
        elif self.get_estimated_heading() == Side.RIGHT:
            return tuple((x+1, y))

    def get_gyro_heading(self):
        return self._gyro.angle()

    def reset_gyro(self, angle):
        self._gyro.reset_angle(angle)
