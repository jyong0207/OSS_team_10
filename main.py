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
ev3.speaker.beep()


threshold = 50
kp = 1.2

# N, E, S, W = 1, 2, 3, 4

# def turn_min(now_dir, target_dir):
#     diff = (target_dir - now_dir) % 4
#     angle = [0, 90, 180, -90][diff]
#     robot.turn(angle)
#     return target_dir

# def follow_line_one_cell():
#     while(1):
#         left_reflection = cs_l.reflection()
#         right_reflection = cs_r.reflection()
#         if right_reflection < 30:
#             robot.stop()
#             break
#         else:
#             error = left_reflection - threshold
#             turn_rate = kp * error
#             robot.drive(100, turn_rate)
#         wait(10)

# def move_manhattan(start_xy, goal_xy, now_dir):

#     x, y = start_xy
#     gx, gy = goal_xy
#     dx = gx - x
#     dy = gy - y

#     if dx != 0:
#         target_dir = E if dx > 0 else W
#         now_dir = turn_min(now_dir, target_dir)
#         steps = abs(dx)
#         for _ in range(steps):
#             follow_line_one_cell()
#             x += 1 if target_dir == E else -1


#     if dy != 0:
#         target_dir = N if dx > 0 else S
#         now_dir = turn_min(now_dir, target_dir)
#         steps = abs(dy)
#         for _ in range(steps):
#             follow_line_one_cell()
#             y += 1 if target_dir == N else -1

#     return (x, y), now_dir


# start = (0, 0)
# goal = (3, 2)
# now_dir = N
# (fx, fy), now_dir = move_manhattan(start, goal, now_dir)

# ev3.screen.clear()
# ev3.screen.print("Done", fx, fy, 'dir', now_dir)
# ev3.speaker.beep()

left = Motor(Port.B)
right = Motor(Port.A)
arm = Motor(Port.C)

cs_l = ColorSensor(Port.S4)
cs_r = ColorSensor(Port.S1)
cs_u = ColorSensor(Port.S2)
ult = UltrasonicSensor(Port.S3)

robot = DriveBase(left, right, 55.5, 114)

from collections import deque

MAP = [
    ".....",
    ".#...",
    ".#..."
]

G = [[1 if c== "#" else 0 for c in r] for r in MAP]

H, W = len(G), len(G[0])

S, E = (0, 0), (4, 4)

def bfs(s, g):
    dist = [[None]*W for _ in range(H)]
    prev = [[None]*W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0
    while q:
        x, y = q.popleft()
        if (x, y) == g:
            break
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= H and not G[ny][nx] and dist[ny][nx] is None:
                dist[ny][nx] = dist[y][x] + 1
                prev[ny][nx] = (x, y)
                q.append((nx, ny))

    path = []
    if dist[g[1]][g[0]] is not None:
        p = g
        while p:
            path.append(p)
            p = prev[p[1]][p[0]]
        path.reverse()
    return path, dist


def left_line_following(speed, kp):
    threshold = 50
    left_reflection = cs_l.reflection()
    error = left_reflection - threshold
    turn_rate = kp *error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    threshold = 50
    right_reflection = cs_r.reflection()
    error = right_reflection - threshold
    turn_rate = -kp *error
    robot.drive(speed, turn_rate)

def n_move(n, direction = "right"):
    for _ in range(n):
        if direction == "right":
            while cs_r.reflection() > 30:
                left_line_following(100, 1.2)
            while cs_r.reflection() <= 30:
                right_line_following(100, 1.2)
        elif direction == "left":
            while cs_l.reflection() > 30:
                right_line_following(100, 1.2)
            while cs_r.reflection() <= 30:
                left_line_following(100, 1.2)
    robot.stop()

def grab_object():
    arm.run_until_stalled(200, then = Stop.COAST, duty_limit = 50)

def release_object():
    arm.run_until_stalled(-200, then = Stop.COAST, duty_limit = 50)

release_object()
robot.straight(100)
n_move(1, direction="left")
while ult.distance() > 120:
    print(ult.distance())
    left_line_following(100, 1.2)
robot.stop()

robot.straight(100)

grab_object()
wait(500)
object_color = cs_u.color()
print("Detected object color:", object_color)

if object_color == Color.RED:
    robot.turn(180)
    n_move(1, direction="left")
    robot.straight(80)
    robot.turn(-90)
    n_move(1, direction="right")
    robot.straight(30)
    robot.turn(90)
    n_move(1, direction="right")
    robot.straight(30)
    release_object()

elif object_color == Color.BLUE:
    robot.turn(180)
    n_move(1, direction="left")
    robot.straight(80)
    robot.turn(-90)
    n_move(2, direction="right")
    robot.straight(30)
    robot.turn(90)
    n_move(1, direction="right")
    robot.straight(30)
    release_object()
