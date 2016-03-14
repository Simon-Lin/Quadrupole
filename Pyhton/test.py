import Quad
from visual import *

rate(20)
sensor = Quad.Sensor(20)
sensor.initialize()
v_null = Quad.Vector3D()
v_normal = Quad.Vector3D()

mybox = box(axis=(0,1,0), length=1, height=5, width=7)
while True:
    sensor.getMotionData(v_null, v_null, v_null, v_null, v_normal)
    v_normal.normalize()
    mybox.axis = (v_normal.x, v_normal.z, v_normal.y)
