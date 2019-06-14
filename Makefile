output: opengl_main.cpp
	g++ -std=c++14 opengl_main.cpp -framework GLUT -framework OpenGL -Wno-deprecated-declarations
