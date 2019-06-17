#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file;
    cin>>file;
    graph test(file);
    test.printInfo();
    cout<<test.strongly_connected() << endl;

    auto d = test.dijkstra('A');
    auto it = d.begin();
    for(auto element : d){
        cout << "Distancia a " << element.first << ": " << element.second.first << " (parent " << element.second.second << ")\n";
    }
    
    auto test2 = test.aStar('A', 'H');
    test2->printInfo();
    cout << endl;
    test2->DFS('A');

    graph* temp = test.BFS('A');
    temp->BFS('A');
    
    auto targets = new vector<char>;
    targets->push_back('H');
    targets->push_back('A');

    unordered_map<char, graph*> * testMap = test.parallel_aStar('A', targets);
    
    cout << "Termina el paralelo"<< endl;

    temp = (*testMap)['A'];
    temp->BFS('A');
    temp = ((*testMap)['H'])->BFS('A');

    return EXIT_SUCCESS;
}
