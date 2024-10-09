#!/usr/bin/env pybricks-micropython

from robot import Robot

if __name__ == '__main__':
    robot = Robot()
    #robot.testes()

    robot.find_reference_corner()
    while True:
        place = None
        while place is None:
            place = robot.take_person()
        robot.deliver_person(place)
