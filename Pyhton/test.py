import Quad
from visual import *

sensor = Quad.Sensor(30)
sensor.initialize()
v_null = Quad.Vector3D()
v_gdir = Quad.Vector3D()
v_omega = Quad.Vector3D()
yaw = 0

mybox = box(axis=(0,1,0), length=1, height=5, width=8)
while True:
    rate(30)
    sensor.getMotionData(v_gdir, v_null, v_null, v_omega, v_null)
    g_dir = vector(v_gdir.x, v_gdir.z, v_gdir.y)
    omega = vector(v_omega.x, v_omega.z, v_omega.y)
    g_dir = g_dir.norm()
    yaw = yaw - v_omega.z*3.1416/180.0
    
    normal = vector(0,1,0)
    front = vector(0,0,1)
    front = front.rotate(diff_angle(normal, g_dir), cross(g_dir, normal))
    normal = normal.rotate(diff_angle(normal, g_dir), cross(g_dir, normal))
    front = front.rotate(yaw, vector(0,1,0))
    normal = normal.rotate(yaw, vector(0,1,0))
    mybox.axis = normal
    mybox.up = cross(normal, front)
    
