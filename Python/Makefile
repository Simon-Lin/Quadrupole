OBJS = ../Sensor.o ../Drivers/I2Cdev.o ../Drivers/MPU6050.o ../Drivers/BMP085.o ../Drivers/PCF8591.o

all: Quad.so

Quad.so: wrapper.o $(OBJS)
	g++ -shared -Wl,--export-dynamic -o Quad.so wrapper.o $(OBJS) -lboost_python-py27 -lbcm2835 -lpthread -lm

wrapper.o: wrapper.cpp
	g++ -c wrapper.cpp -ansi -O3 -I/usr/include/python2.7

Vector3D.o: Vector3D.cpp
	g++ -c Vector3D.cpp -ansi -O3

$(OBJS): $(OBJS:.o=.cpp)
	cd .. && make

clean:
	rm Quad.so wrapper.o
