OBJS = ../Vector3D.o ../Sensor.o ../Drivers/I2Cdev.o ../Drivers/MPU6050.o ../Drivers/BMP085.o

all: Quad.so

Quad.so: wrapper.o $(OBJS)
	g++ -shared -Wl,--export-dynamic -o Quad.so wrapper.o $(OBJS) -l boost_python -l bcm2835

wrapper.o: wrapper.cpp
	g++ -c wrapper.cpp -ansi -O3 -I/usr/include/python2.7

$(OBJS): $(OBJS:.o=.cpp)
	cd .. && make

clean:
	rm Quad.so wrapper.o
