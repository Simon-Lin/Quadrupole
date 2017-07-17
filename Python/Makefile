OBJS = ../Sensor.o ../Drivers/I2Cdev.o ../Drivers/MPU6050.o ../Drivers/BMP085.o ../Drivers/PCF8591.o

all: pipe

pipe: pipe.cpp $(OBJS)
	g++ -o pipe pipe.cpp $(OBJS) -ansi -Wall -O0 -g3 -lbcm2835 -lpthread -lm

$(OBJS): $(OBJS:.o=.cpp)
	cd .. && make
