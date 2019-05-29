//#include <GL/glut.h>
#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    graph test("graph.txt");
    cout<<"Done"<<endl;

    test.printInfo();

    cout<<endl<<"BFS:"<<endl;
    test.BFS(0);
    cout<<endl<<"DFS:"<<endl;
    test.DFS(0);

    /*test.addEdge(nodes[1], nodes[0]); //connect B and A
    test.addEdge(nodes[2], nodes[0]); //connect C and A*/
    //test.addEdge(nodes[1], nodes[2]); //connect B and C


    //if(test.getType(nodes[1]) == hoja) cout << "Hoja\n";

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
    //cout << test.isStronglyConnected() << endl;
    return EXIT_SUCCESS;
}
