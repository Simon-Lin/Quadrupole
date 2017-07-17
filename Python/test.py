from __future__ import division
from subprocess import Popen, PIPE
from visual import *

pipe = Popen('./pipe', stdout=PIPE, bufsize=-1)

# [gx, gy, gz, wx, wy, wz, ax, ay, az]
state = [0, 0, 1, 0, 0, 0, 0, 0, 0]
yaw = 0
mybox = box(axis=(0,1,0), length=1, height=5, width=8)
n = 0

for line in iter(pipe.stdout.readline, b''):
    if line.rstrip() == 'PIPEDATA:':
        break

for line in iter(pipe.stdout.readline, b''):
    rate(100)
    print line

while True:
    for line in iter(pipe.stdout.readline, b''):
        if line.rstrip() == 'PIPEDATA:':
            n = 0
            break
        else:
            state[n] = float(line.rstrip())
            n = n+1

    for i in state:
        print i
    print "=================\n"
    g_dir = vector(state[0], state[2], -state[1])
    omega = vector(state[3], state[5], -state[4])
    dyaw = dot(-g_dir, omega)
    yaw = yaw + dyaw*3.1416/180.0/10
    normal = vector(0,1,0)
    front = vector(0,0,-1)
    front = front.rotate(diff_angle(normal, -g_dir), cross(-g_dir, normal))
    normal = normal.rotate(diff_angle(normal, -g_dir), cross(-g_dir, normal))
    front = front.rotate(yaw, vector(0,1,0))
    normal = normal.rotate(yaw, vector(0,1,0))
    mybox.axis = normal
    mybox.up = cross(normal, front)
