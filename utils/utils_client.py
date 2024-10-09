from pybricks.parameters import Color


def rgb_to_color_claw(rgb):
    r = rgb[0]
    g = rgb[1]
    b = rgb[2]

    if (r < 20) and (g < 20) and (b < 20):
        return Color.BROWN
    elif r > g and r > b:
        return Color.RED
    elif g > r and g > b:
        return Color.GREEN
    if b > r and b > g:
        return Color.BLUE

    return None


def color_to_int_claw(color: Color) -> int:
    colors = {
        Color.RED: 1,
        Color.BLUE: 2,
        Color.BROWN: 3,
        Color.GREEN: 4,
        None: 0
    }
    result = colors[color]
    return colors[color]
