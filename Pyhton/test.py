import Quad
from visual import *

sensor = Quad.Sensor(50)
sensor.initialize()
v_null = Quad.Vector3D()
v_gdir = Quad.Vector3D()
v_omega = Quad.Vector3D()
yaw = 0

mybox = box(axis=(0,1,0), length=1, height=5, width=8)
while True:
    rate(50)
    sensor.getMotionData(v_null, v_null, v_null, v_omega, v_gdir)
    g_dir = vector(v_gdir.x, v_gdir.z, -v_gdir.y)
    omega = vector(v_omega.x, v_omega.z, -v_omega.y)
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
