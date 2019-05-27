//#include <GL/glut.h>
#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    graph test;
    vector<Node<graph>*> nodes;
    for(char c = 'A'; c<'E'; c++)
        nodes.push_back(test.addVertex(c));
    test.addEdge(nodes[0], nodes[1]);
    test.addEdge(nodes[0], nodes[2]);
    test.DFS(nodes[0]);
    test.BFS(nodes[0]);
    test.MST_Prim(nodes[0]);
    return EXIT_SUCCESS;
}
