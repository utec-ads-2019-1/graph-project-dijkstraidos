//#include <GL/glut.h>
#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    graph test("graph.txt");
    cout<<endl<<"BFS:"<<endl;
    test.BFS(0);
    cout<<endl<<"DFS:"<<endl;
    test.DFS(0);
    return EXIT_SUCCESS;
}
