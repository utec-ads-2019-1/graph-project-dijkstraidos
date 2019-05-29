//#include <GL/glut.h>
#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    graph test(true);
    //graph test;
    vector<Node<graph>*> nodes;
    for(char c = 'A'; c<'E'; c++)
        nodes.push_back(test.addVertex(c));

    cout << test.addEdge(nodes[0], nodes[1]); //connect A and B
    test.addEdge(nodes[0], nodes[2]); //connect A and C
    test.addEdge(nodes[0], nodes[3]); //connect A and D
    test.addEdge(nodes[1], nodes[2]); //connect B and C

    //test.removeVertex(nodes[1]);
    /*test.removeEdge(nodes[0], nodes[1]);

    if(!test.findVertex(nodes[0])) cout << "No se encontró el vértice B\n";

    if(!test.findEdge(nodes[1], nodes[0])) cout << "No es encontró la arista entre A y B\n";*/

    /*if(test.isDense(0.7)){
        cout << "es denso" << endl;
    }else{
        cout << "no es denso" << endl;
    }

    cout << test.countEdges() << endl;*/

    //cout << test.nonDirected_isConnected() << endl;

    /*test.printInfo();
    test.printDegrees();
    test.printTypes();
*/
    /*if(test.isBipartite()) cout << "Es bipartito" << endl;
    else cout << "No es bipartito" << endl;*/

    cout << "Depth First Search:\n";
    test.DFS(nodes[0]);
    cout << "\nBreadth First Search:\n";
    test.BFS(nodes[0]);
    return EXIT_SUCCESS;
}
