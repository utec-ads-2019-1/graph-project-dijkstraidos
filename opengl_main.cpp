#include <bits/stdc++.h> 
#include <GLUT/glut.h> 
#include <OpenGl/glu.h>
#include <OpenGL/gl.h>
#include <dirent.h>

#include "graph.h"

#define pi 3.1415926535897932384626433832795

using namespace std;

graph* mg;
string mg_name;
stack<graph*> undo;
vector<graph*> loaded_graphs;
vector<string> loaded_graph_names;
void* currentfont;

void draw_circle(double x, double y, double r){
    glBegin(GL_LINE_LOOP);
    for (double i = 0; i<2*pi; i+=0.001){
        double tx = x + r*cos(i);
        double ty = y + r*sin(i);
        glVertex2i(tx, ty);
    }
    glEnd();
}

void draw_string(double x, double y, char *str){
	glRasterPos2f(x,y);

    for(char* c = str; *c != '\0'; c++){
		glutBitmapCharacter(currentfont, *c);
	}
}

void draw_string(double x, double y, string str){
	glRasterPos2f(x,y);
	for(char c : str){
		glutBitmapCharacter(currentfont, c);
	}
}

void draw_edges(pair<double, double> a, pair<double, double> b, char* w){
    double ang = atan2(a.second-b.second, a.first - b.first);
    double x0, y0, x1, y1;
    if(a.first < b.first && cos(ang)>0){
        x0 = a.first + 20*cos(ang);
        x1 = b.first - 20*cos(ang);
    }
    else{
        x0 = a.first - 20*cos(ang);
        x1 = b.first + 20*cos(ang);
    }
        
    if(a.second < b.second && sin(ang)>0){
        y0 = a.second + 20*sin(ang);
        y1 = b.second - 20*sin(ang);
    }
    else{
        y0 = a.second - 20*sin(ang);
        y1 = b.second + 20*sin(ang);
    }

    if(mg->isWeighted()){
        double xm = (x0 + x1)/2.0;
        double ym = (y0 + y1)/2.0;
        double xtx = xm + 10*cos(ang);
        double ytx = ym + 10*sin(ang);
        draw_string(xtx, ytx, w);
    }

    glBegin(GL_LINES);
    glVertex2i(x0, y0);
    glVertex2i(x1, y1);
    if(mg->isDirected()){
        double tx0 = x1 + 10*cos(ang+0.4);
        double tx1 = x1 + 10*cos(ang-0.4);
        double ty0 = y1 + 10*sin(ang+0.4);
        double ty1 = y1 + 10*sin(ang-0.4);
        glVertex2i(tx0, ty0);
        glVertex2i(x1, y1);
        glVertex2i(tx1, ty1);
        glVertex2i(x1, y1);
    }
    glEnd();
}

void draw_graph(){
    typedef typename graph::N N;
    unordered_map<N, pair<double, double>> nodes = mg->getNodesOGL();
    vector<pair<pair<N, N>, int>> edges = mg->getEdgesOGL();

    for(auto xy : nodes){
        draw_circle(xy.second.first, xy.second.second, 20);
        char tmp[2] = {xy.first, '\0'};
        draw_string(xy.second.first - 5, xy.second.second - 5, tmp);
    }

    for(pair<pair<N, N>, int> edg : edges){
        char w[15];
        sprintf(w, "%d", edg.second);
        draw_edges(nodes[edg.first.first], nodes[edg.first.second], w);
    }
}

void load_graphs(){
    DIR *dir;
    struct dirent *ent;
    dir = opendir ("./tests");
    while ((ent = readdir (dir))) {
       if(ent->d_name[0] == '.') continue;
       loaded_graph_names.push_back(ent->d_name);
    }
    closedir (dir);

    sort(loaded_graph_names.begin(), loaded_graph_names.end());
    for(string str : loaded_graph_names){
        string filename = "tests/" + str;
        loaded_graphs.push_back(new graph(filename));
    }
}

void myInit (void) { 
    glClearColor(0.0, 0.0, 0.0, 1.0); 
      
    glColor3f(0.0, 1.0, 0.0); 
      
    glPointSize(1.0); 
    glMatrixMode(GL_PROJECTION);  
    glLoadIdentity(); 
      
    gluOrtho2D(0, 1200, 0, 800); 
} 

void update(){
    string command_string, temp;
    getline(cin, command_string);

    stringstream command(command_string);
    vector<string> command_args;

    while(command>>temp){
        command_args.push_back(temp);
    }

    if(command_args[0] == "change"){
        int op = stoi(command_args[1]);
        mg = loaded_graphs[op];
        mg_name = loaded_graph_names[op];
    }

    else if(command_args[0] == "dfs"){
        mg = mg->DFS(command_args[1][0]);
        mg_name = mg_name + " - DFS";
    }

    else if(command_args[0] == "bfs"){
        mg = mg->BFS(command_args[1][0]);
        mg_name = mg_name + " - BFS";
    }

    else if(command_args[0] == "prim"){
        mg = mg->primMST(command_args[1][0]);
        mg_name = mg_name + " - PRIM";
    }

    else if(command_args[0] == "kruskal"){
        mg = mg->kruskalMST();
        mg_name = mg_name + " - KRUSKAL";
    }

    else{
        cout<<"Not a valid command!"<<endl;
    }

    glutPostRedisplay();
}

void draw_screen(){
    glBegin(GL_LINES);
    glVertex2i(900, 0);
    glVertex2i(900, 800);
    glEnd();

    draw_string(920, 760, mg_name);

    glBegin(GL_LINES);
    glVertex2i(900, 730);
    glVertex2i(1200, 730);
    glEnd();

    draw_string(920, 700, "LOADED GRAPHS:");

    int dy = -30;
    for(string name : loaded_graph_names){
        if(name == mg_name)
            draw_string(920, 700 + dy, "> " + name);
        else
            draw_string(920, 700 + dy, name);
        dy -= 30;
    }
}
  
void display(){
    glClear(GL_COLOR_BUFFER_BIT); 
    draw_graph();
    draw_screen();
    glFlush(); 
}

int main (int argc, char** argv) { 
    glutInit(&argc, argv); 
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB); 
    string filename = "tests/test6.txt";
    mg_name = "test6.txt";
      
    // giving window size in X- and Y- direction 
    glutInitWindowSize(1200, 800); 
    glutInitWindowPosition(500, 400); 
      
    // Giving name to window 
    glutCreateWindow("Circle Drawing"); 
    myInit(); 
    //glutTimerFunc( 0, timer, 0 );

    mg = new graph(filename);
    currentfont = GLUT_BITMAP_9_BY_15;
    load_graphs();

    glutIdleFunc(update); 
    glutDisplayFunc(display); 
    glutMainLoop(); 
} 
