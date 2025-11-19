#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# Write your program here.
left = Motor(Port.B)
right = Motor(Port.A)

cs_l = ColorSensor(Port.S4)
cs_r = ColorSensor(Port.S1)

robot = DriveBase(left, right, 55.5, 104)

threshold = 50
kp = 1.2

now_dir = 1
target_dir = 4

target_cor= [1, 2, 3, 1, 2, 1]

ev3.speaker.beep()

for i in range(target_cor[0]):
    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)

robot.drive(200, 0)
wait(10)
direction = (target_dir - now_dir) % 4
now_dir = target_dir
target_dir = 1
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for j in range(target_cor[1]):
    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)
        
robot.drive(200, 0)
wait(10)
direction = (target_dir - now_dir) % 4
now_dir = target_dir
target_dir = 2
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for k in range(target_cor[2]):
    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)

robot.drive(200, 0)
wait(10)
direction = (target_dir - now_dir) % 4
now_dir = target_dir
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for l in range(target_cor[3]):
    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)

robot.drive(200, 0)
wait(10)
direction = (target_dir - now_dir) % 4
now_dir = target_dir
target_dir = 1
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for m in range(target_cor[4]):
    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_l.reflection()
        right_reflection = cs_r.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)

robot.drive(200, 0)
wait(10)
direction = (target_dir - now_dir) % 4
now_dir = target_dir
turn_table = [0, 90, 180, -90]
angle = turn_table[direction]
robot.turn(angle)
wait(10)

for l in range(target_cor[5]):
    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection < 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)    

    while(1):
        left_reflection = cs_r.reflection()
        right_reflection = cs_l.reflection()
        if right_reflection > 40:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)



# 이게 두칸 가는거
        
# while(1):
#     left_reflection = cs_l.reflection()
#     right_reflection = cs_r.reflection()
#     if right_reflection < 30:
#         robot.stop()
#         break
#     else:
#         error = left_reflection - threshold
#         turn_rate = kp * error
#         robot.drive(100, turn_rate)
#     wait(10)

# while(1):
#     left_reflection = cs_l.reflection()
#     right_reflection = cs_r.reflection()
#     if right_reflection > 30:
#         robot.stop()
#         break
#     else:
#         error = left_reflection - threshold
#         turn_rate = kp * error
#         robot.drive(100, turn_rate)
#     wait(10)    


# while(1):
#     left_reflection = cs_l.reflection()
#     right_reflection = cs_r.reflection()
#     if right_reflection < 30:
#         robot.stop()
#         break
#     else:
#         error = left_reflection - threshold
#         turn_rate = kp * error
#         robot.drive(100, turn_rate)
#     wait(10)

# left.stop()
# right.stop()

# line_sensor = ColorSensor(Port.S1)
# robot = DriveBase(left, right, 55.5, 104)
# BLACK = 9
# WHITE = 85
# threshold = (BLACK + WHITE) / 2

# DRIVE_SPEED = 100
# PROPORTIONAL_GAIN = 1.2

# while True:
#     deviation = line_sensor.reflection() - threshold
#     turn_rate = PROPORTIONAL_GAIN * deviation
#     robot.drive(DRIVE_SPEED, turn_rate)
#     wait(10)    

# robot.drive(100,820)
# wait(1000)

# robot.drive(100,0)
# wait(2000)
# robot.turn(90)

# robot.drive(100,0)
# wait(2000)
# robot.turn(90)

# robot.drive(100,0)
# wait(2000)


# robot.stop()
# robot.straight(100)
# robot.turn(360)
# ev3.speaker.beep()

# robot.straight(-100)
# robot.turn(-360)
# ev3.speaker.beep()

# left.run_target(150, 360)
# left.run(-150)
# right.run(-150)

# wait(2000)