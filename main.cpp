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
    cout<<endl<<"KMST"<<endl;
    graph KMST = test.kruskalMST();
    KMST.BFS(0);



    cout<<endl<<"PMST"<<endl;
    graph PMST = test.MST_Prim(0);


    cout<<endl<<endl<<"BFSMST"<<endl;
    PMST.BFS(0);

    if(!test.findEdge(0, 1)) cout << "No es encontró la arista entre A y B\n";

    cout<<endl;
    test.removeEdge(0, 2);
    test.removeEdge(0, 3);

    if(test.getType(0) == hoja) cout << "Hoja\n";

    test.removeVertex(0);
    cout<<endl<<"BFS:"<<endl;
    test.BFS(0);

    if(test.isDense(0.7)){
        cout << "es denso" << endl;
    }else{
        cout << "no es denso" << endl;
    }

    cout << test.countEdges() << endl;

    return EXIT_SUCCESS;
}
