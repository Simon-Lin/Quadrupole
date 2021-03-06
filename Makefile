
CC = g++
CFLAGS = -ansi -Wall -O0 -g3
LDFLAGS = -lbcm2835 -lncurses -lm -lpthread
OBJ = Sensor.o Controller.o Interface.o
OBJ_DRIV = ./Drivers/I2Cdev.o ./Drivers/BMP085.o ./Drivers/MPU6050.o ./Drivers/PCA9685.o ./Drivers/PCF8591.o

all: test Quadrupole

Quadrupole: Quadrupole.o $(OBJ) $(OBJ_DRIV)
	$(CC) -o $@ $< $(OBJ) $(OBJ_DRIV) $(CFLAGS) $(LDFLAGS)

test: test.o $(OBJ) $(OBJ_DRIV)
	$(CC) -o $@ $< $(OBJ) $(OBJ_DRIV) $(CFLAGS) $(LDFLAGS)

%.o: %.cpp
	$(CC) -c -o $@ $< $(CFLAGS)

clean:
	rm *.o ./Drivers/*.o  Quadrupole test
