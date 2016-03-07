CC=g++
CFLAGS= -ansi -Wall -O0 -g3
LDFLAGS=-l bcm2835 -l m
OBJ=Sensor.o Vector3D.o
OBJ_DRIV= ./Drivers/I2Cdev.o ./Drivers/BMP085.o ./Drivers/MPU6050.o ./Drivers/PCA9685.o

all: test

test: test.o $(OBJ) $(OBJ_DRIV)
	$(CC) -o $@ $< $(OBJ) $(OBJ_DRIV) $(CFLAGS) $(LDFLAGS)

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS) $(LDFLAGS)

clean:
	rm *.o ./Drivers/*.o  test
