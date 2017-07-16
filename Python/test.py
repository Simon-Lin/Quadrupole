import Quad
from visual import *

sensor = Quad.Sensor(50)
sensor.initialize()
yaw = 0
a = Quad.V3f()
w = Quad.V3f()
g = Quad.V3f()

mybox = box(axis=(0,1,0), length=1, height=5, width=8)

while True:
    rate(50)
    Quad.getDataWrapper(sensor, a, w, g)
    g_dir = vector(g.x, g.z, -g.y)
    omega = vector(w.x, w.z, -w.y)
    dyaw = dot(-g_dir, omega)
    yaw = yaw + dyaw*3.1416/180.0/50
    normal = vector(0,1,0)
    front = vector(0,0,-1)
    front = front.rotate(diff_angle(normal, -g_dir), cross(-g_dir, normal))
    normal = normal.rotate(diff_angle(normal, -g_dir), cross(-g_dir, normal))
    front = front.rotate(yaw, vector(0,1,0))
    normal = normal.rotate(yaw, vector(0,1,0))
    mybox.axis = normal
    mybox.up = cross(normal, front)
