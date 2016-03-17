FLAGS= -ansi -Wall -O0 -g3 -lncurses -lm -framework IOKit -framework CoreFoundation -framework OpenGL -framework GLUT -framework ApplicationServices

all: client

client: client.o Vector3D.o 
	g++ -o client client.o Vector3D.o libstem_Gamepad.a $(FLAGS)

client.o: client.cpp
	g++ -c client.cpp $(FLAGS)

Vector3D.o: Vector3D.cpp
	g++ -c Vector3D.cpp $(FLAGS)

