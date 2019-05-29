//#include <GL/glut.h>
#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file = "graph.txt";
    graph test(file);
    cout<<"Done"<<endl;

    test.printInfo();

    cout<<endl<<"BFS:"<<endl;
    test.BFS(0);
    cout<<endl<<"DFS:"<<endl;
    test.DFS(0);

    cout<<endl;
    test.removeEdge(0, 2);
    test.removeEdge(0, 3);

    if(test.getType(0) == hoja) cout << "Hoja\n";

    /*if(!test.findVertex(nodes[0])) cout << "No se encontró el vértice B\n";

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
