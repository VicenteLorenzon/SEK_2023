from pybricks.hubs import EV3Brick

from subsystems.claw import Claw
from subsystems.drive import Drive, DrivePosition, Side
from pybricks.messaging import BluetoothMailboxServer
from pybricks.parameters import Color
from utils.utils_server import calculate_path, Place, get_deliver_positions, run_commands
from pybricks.tools import wait


class Robot:
    def __init__(self):
        server = BluetoothMailboxServer()

        print('waiting for connection...')
        server.wait_for_connection(1)
        print('connected!')

        self.hub = EV3Brick()
        self.drive = Drive(server)
        self.claw = Claw(server)

        self.delivered_people = {
            Place.PARK: 0,
            Place.MUSEUM: 0,
            Place.BAKERY: 0,
            Place.SCHOOL: 0,
            Place.LIBRARY: 0,
            Place.CITY_HALL: 0,
            Place.DRUGSTORE: 0
        }

    def testes(self):
        run_commands(500,
                     lambda: self.drive.drive_straight(250, 200),
                     lambda: self.drive.turn_angle(90),
                     lambda: self.drive.drive_straight(250, 200),
                     lambda: self.drive.turn_angle(90),
                     lambda: self.drive.drive_straight(250, 200),
                     lambda: self.drive.turn_angle(90),
                     lambda: self.drive.drive_straight(250, 200),
                     lambda: self.drive.turn_angle(90))

    def go_to_position(self, target_pos: DrivePosition):
        while True:
            if (self.drive.get_position_x() == target_pos.x) and (self.drive.get_position_y() == target_pos.y):
                if self.drive.get_estimated_heading() != target_pos.heading:
                    wait(250)
                    self.drive.turn_to_side(target_pos.heading)
                self.drive.set_position_x(target_pos.x)
                self.drive.set_position_y(target_pos.y)
                self.drive.set_heading(target_pos.heading)
                return True

            path = calculate_path((self.drive.get_position_x(), self.drive.get_position_y()),
                                  (target_pos.x, target_pos.y),
                                  self.drive.get_prohibited_squares())
            if path is None:  # Se é impossível chegar na localização
                return False

            for (x, y) in path:
                distance = max(abs(self.drive.get_position_x() - x), abs(self.drive.get_position_y() - y))

                if (self.drive.get_position_y() == 0) and (self.drive.get_position_x() == 0) and (y == 0):
                    wait(250)
                    self.drive.drive_straight_squares(-distance, explicit_direction=self.drive.get_estimated_heading())
                else:
                    if self.drive.get_estimated_heading() != self.drive.get_side_of_adjacent_square((x, y)):
                        wait(250)
                        self.drive.turn_to_side(self.drive.get_side_of_adjacent_square((x, y)))

                    if self.drive.is_seeing_obstacle_far():
                        self.drive.add_obstacle(self.drive.get_square_ahead())
                        break

                    wait(250)
                    self.drive.drive_straight_squares(distance, explicit_direction=self.drive.get_estimated_heading())

    def deliver_person(self, place: Place):
        entrance_positions = get_deliver_positions(place, self.delivered_people[Place.PARK])

        if place == Place.PARK:
            if self.delivered_people[place] == 0:
                turn, distance = 0, 175
            elif self.delivered_people[place] == 1:
                turn, distance = 0, 175
            else:
                turn, distance = 0, 175
        elif place in [Place.DRUGSTORE, Place.CITY_HALL]:
            if self.delivered_people[place] == 0:
                turn, distance = 0, 300
            elif self.delivered_people[place] == 1:
                turn, distance = 0, 250
            else:
                turn, distance = 0, 175
        else:
            if self.delivered_people[place] == 0:
                turn, distance = 0, 300
            elif self.delivered_people[place] == 1:
                turn, distance = 0, 250
            else:
                turn, distance = 0, 175

        for position in entrance_positions:
            if self.go_to_position(position):
                break
        # Nesse ponto o robô deve estar na posição e virado pra entrada
        # Entrega a pessoa no local
        self.drive.align_on_color([Color.BLUE, Color.BLACK, Color.YELLOW])
        wait(250)
        self.drive.drive_straight(100, distance=-distance)
        self.claw.open()
        self.drive.drive_straight(100, distance=distance - 40)

        self.delivered_people[place] += 1

        # Volta pro canto (0, 0), virado pro vermelho
        self.go_to_position(DrivePosition(1, 0, Side.FRONT))
        self.drive.align_on_color([Color.BLUE])
        wait(250)
        self.drive.reset_gyro(0)
        wait(250)
        self.drive.drive_straight(150, distance=-40)
        wait(250)
        self.drive.turn_to_side(Side.LEFT)

        if sum(self.delivered_people.values()) < 2:
            wait(500)
            self.drive.drive_straight(100, stop_conditions=[self.drive.any_sensor_sees_color], explicit_direction=Side.LEFT),
            self.drive.align_on_color([Color.RED]),
            wait(250)
            self.drive.reset_gyro(Side.LEFT)

    def take_person(self) -> None:
        while True:
            run_commands(500,
                         lambda: self.drive.drive_straight(-150, stop_conditions=[self.drive.is_seeing_person,
                                                                                  self.drive.any_sensor_sees_color],
                                                           explicit_direction=Side.LEFT))
            if self.drive.is_seeing_person():
                break
            elif self.drive.any_sensor_sees_color([Color.RED]):
                self.drive.drive_straight(150, distance=300)
                self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color])
            else:
                self.drive.drive_straight(-150, stop_conditions=[self.drive.is_seeing_person,
                                                                 self.drive.any_sensor_sees_color],
                                          explicit_direction=Side.LEFT)

        distance, is_adult = self.drive.measure_tube()
        distance += 20
        self.drive.drive_straight(100, distance=55, explicit_direction=Side.LEFT)
        run_commands(250,
                     lambda: self.drive.turn_to_side(Side.BACK),
                     lambda: self.drive.drive_straight(50, distance=-distance, explicit_direction=Side.BACK))

        took_person = self.claw.grab()
        self.drive.drive_straight(150, distance=distance + 20, explicit_direction=Side.BACK)
        run_commands(250,
                     lambda: self.drive.turn_to_side(Side.LEFT),
                     lambda: self.drive.drive_straight(175, stop_conditions=[self.drive.any_sensor_sees_color],
                                                       explicit_direction=Side.LEFT))
        self.drive.align_on_color([Color.RED])
        self.drive.align_on_color([Color.RED])
        wait(250)
        self.drive.reset_gyro(Side.LEFT)
        wait(250)
        self.drive.drive_straight(150, distance=-35, explicit_direction=Side.LEFT)

        self.drive.set_position_x(0)
        self.drive.set_position_y(0)
        self.drive.set_heading(Side.LEFT)

        if not took_person:
            self.claw.open()
            return None

        color = self.claw.claw_sensor_color()

        # Retorna o lugar que tem que levar a pessoa
        if is_adult:
            if color == Color.BLUE:
                return Place.MUSEUM
            elif color == Color.RED:
                return Place.DRUGSTORE
            elif color == Color.BROWN:
                return Place.BAKERY
            elif color == Color.GREEN:
                return Place.CITY_HALL
        else:
            if color == Color.BLUE:
                return Place.SCHOOL
            elif color == Color.BROWN:
                return Place.LIBRARY
            elif color == Color.GREEN:
                return Place.PARK

    def find_reference_corner(self):
        def found_black_yellow():
            self.drive.align_on_color([Color.BLACK, Color.YELLOW])
            run_commands(500,
                         lambda: self.drive.drive_straight(150, distance=-50),
                         lambda: self.drive.turn_angle(90))

        def found_blue():
            self.drive.align_on_color([Color.BLUE])
            wait(250)
            self.drive.reset_gyro(0)
            wait(250)
            run_commands(500,
                         lambda: self.drive.drive_straight(150, distance=-40),
                         lambda: self.drive.turn_angle(-90),
                         lambda: self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color], explicit_direction=Side.LEFT))
            self.drive.align_on_color([Color.RED])
            wait(250)
            self.drive.reset_gyro(Side.LEFT)
            self.drive.set_position_x(0)
            self.drive.set_position_y(0)
            self.drive.set_heading(Side.LEFT)

        def found_red():
            self.drive.align_on_color([Color.RED])
            run_commands(500,
                         lambda: self.drive.drive_straight(150, distance=-30),
                         lambda: self.drive.turn_angle(90),
                         lambda: self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color]))

            if self.drive.any_sensor_sees_color([Color.BLUE]):
                found_blue()
                return

            yellow_at_right = self.drive.align_on_color([Color.BLACK, Color.YELLOW])

            run_commands(500,
                         lambda: self.drive.drive_straight(150, distance=-40),
                         lambda: self.drive.turn_angle(-180),
                         lambda: self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color]))

            if self.drive.any_sensor_sees_color([Color.BLUE]):
                found_blue()
                return

            yellow_at_left = self.drive.align_on_color([Color.BLACK, Color.YELLOW])

            if yellow_at_right and yellow_at_left:
                run_commands(500,
                             lambda: self.drive.drive_straight(150, distance=-40))
                self.drive.set_position_y(4)
                self.drive.set_position_x(0)
                self.drive.set_heading(Side.BACK)
                self.drive.reset_gyro(Side.BACK)
                self.go_to_position(DrivePosition(1, 0, Side.FRONT))
                self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color])
                found_blue()
                return

            self.drive.align_on_color([Color.BLACK, Color.YELLOW])
            run_commands(500,
                         lambda: self.drive.drive_straight(150, distance=-40),
                         lambda: self.drive.turn_angle(-90))

            self.drive.set_position_x(4)
            self.drive.set_position_y(2)
            self.drive.set_heading(Side.LEFT)
            self.drive.reset_gyro(Side.LEFT)

            if self.go_to_position(DrivePosition(3, 0, Side.FRONT)):
                self.drive.drive_straight(100, stop_conditions=[self.drive.any_sensor_sees_color,
                                                                self.drive.is_seeing_obstacle_very_close])
                if self.drive.any_sensor_sees_color([Color.BLUE]):
                    found_blue()
                    return
                elif self.drive.any_sensor_sees_color([Color.BLACK]):
                    self.drive.align_on_color([Color.BLACK])
                    run_commands(250,
                                 lambda: self.drive.reset_gyro(Side.BACK),
                                 lambda: self.drive.drive_straight(150, -50))
                    self.drive.set_position_x(1)
                    self.drive.set_position_y(4)
                    self.drive.set_heading(Side.BACK)
                    self.go_to_position(DrivePosition(1, 0, Side.FRONT))
                    self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color])
                    found_blue()
                    return
                else:
                    wait(500)
                    self.drive.drive_straight(150, distance=-180)
                    self.drive.set_position_y(2)
                    self.drive.set_position_x(3)
                    self.drive.set_heading(Side.FRONT)
                    self.drive.add_obstacle(self.drive.get_square_ahead())
                    self.go_to_position(DrivePosition(1, 0, Side.FRONT))
                    self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color])
                    found_blue()
                    return

            else:
                run_commands(500,
                             lambda: self.drive.turn_angle(180),
                             lambda: self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color]))
                found_blue()
                return

        while True:
            self.drive.drive_straight(150, stop_conditions=[self.drive.any_sensor_sees_color,
                                                            self.drive.is_seeing_obstacle_very_close])
            if self.drive.any_sensor_sees_color([Color.BLACK, Color.YELLOW]):
                found_black_yellow()
            elif self.drive.any_sensor_sees_color([Color.RED]):
                found_red()
                break
            elif self.drive.any_sensor_sees_color([Color.BLUE]):
                found_blue()
                break
            elif self.drive.is_seeing_obstacle_very_close():
                run_commands(500,
                             lambda: self.drive.drive_straight(150, distance=-180),
                             lambda: self.drive.turn_angle(90))
