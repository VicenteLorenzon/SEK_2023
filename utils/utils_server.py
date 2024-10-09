from pybricks.parameters import Color
from subsystems.drive import DrivePosition, Side
from constants import POSSIBLE_OBSTACLE_SQUARES
from pybricks.tools import wait


class Place:
    BAKERY = 0
    SCHOOL = 1
    DRUGSTORE = 2
    CITY_HALL = 3
    MUSEUM = 4
    LIBRARY = 5
    PARK = 6


def rgb_to_color(rgb):
    r = rgb[0]
    g = rgb[1]
    b = rgb[2]

    if (r > 50) and (g > 50) and (b > 50):
        return Color.WHITE
    if (r < 30) and (g < 30) and (b < 30):
        return Color.BLACK
    if (r > 45) and (g > 45) and (b < 35):
        return Color.YELLOW
    if (r > 40) and (g < 25) and (b < 25):
        return Color.RED
    if (r < 30) and (g < 30) and (b > 35):
        return Color.BLUE

    return Color.WHITE


def calculate_path(initial, final, obstacles):
    def is_valid_square(x, y):
        return 0 <= x < 5 and 0 <= y < 5

    def get_adjacent_squares(x, y):
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def optimize_path(points):
        optimized_path = []
        #  points = [points[0]] + points
        for i in range(1, len(points)-1):
            if points[i + 1] in POSSIBLE_OBSTACLE_SQUARES:
                optimized_path.append(points[i])

            elif not ((points[i - 1][0] == points[i][0] == points[i + 1][0]) or
                      (points[i - 1][1] == points[i][1] == points[i + 1][1])):
                optimized_path.append(points[i])

        optimized_path.append(points[-1])
        return optimized_path

    if initial == final:
        return [initial]

    queue = [(initial, [])]
    visited = {initial}

    while queue:
        current_square, path = queue.pop(0)

        for neighbor in get_adjacent_squares(*current_square):
            if neighbor in obstacles or neighbor in visited or not is_valid_square(*neighbor):
                continue

            new_path = path + [current_square]
            if neighbor == final:
                return optimize_path(new_path + [final])

            queue.append((neighbor, new_path))
            visited.add(neighbor)

    return None


def get_deliver_positions(place, park_delivered_people) -> [DrivePosition]:
    entrances = {
        Place.PARK: [(0, 4, Side.FRONT), (2, 4, Side.FRONT), (4, 4, Side.FRONT)],
        Place.BAKERY: [(1, 3, Side.RIGHT), (0, 4, Side.BACK)],
        Place.SCHOOL: [(1, 1, Side.RIGHT), (0, 2, Side.BACK)],
        Place.LIBRARY: [(4, 0, Side.FRONT), (3, 1, Side.LEFT)],
        Place.CITY_HALL: [(1, 1, Side.LEFT), (3, 1, Side.RIGHT)],
        Place.DRUGSTORE: [(2, 2, Side.FRONT), (2, 4, Side.BACK)],
        Place.MUSEUM: [(3, 3, Side.LEFT), (4, 2, Side.FRONT)]
    }

    if park_delivered_people == 1:
        entrances[Place.PARK] = [(2, 4, Side.FRONT), (4, 4, Side.FRONT), (0, 4, Side.FRONT)]
    elif park_delivered_people == 2:
        entrances[Place.PARK] = [(4, 4, Side.FRONT), (0, 4, Side.FRONT), (2, 4, Side.FRONT)]

    return [DrivePosition(i[0], i[1], i[2]) for i in entrances[place]]


def int_to_color_claw(i: int) -> Color:
    colors = {
        0: None,
        1: Color.RED,
        2: Color.BLUE,
        3: Color.BROWN,
        4: Color.GREEN
    }

    return colors[i]


def run_commands(delay, *functions):
    for function in functions:
        wait(delay)
        function()
