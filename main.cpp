#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file;
    cin>>file;
    //file = "test4.txt";
    graph test(file);
    test.printInfo();
    cout<<test.strongly_connected() << endl;

    auto d = test.BellmanFord('A');
    for(auto element : d){
        cout << "Camino a " << element.first << ": (peso " << (element.second)->graphWeight() <<")" << endl;
        element.second->BFS('A');
    }
    
    // auto d = test.dijkstra('A');
    // for(auto element : d){
    //     cout << "Camino a " << element.first << ": (peso " << (element.second)->graphWeight() <<")" << endl;
    //     element.second->BFS('A');
    // }
    
    // auto test2 = test.aStar('A', 'Z');
    // test2->printInfo();
    // cout << endl;
    // cout <<"A *: (peso" << test2->graphWeight() << ")" << endl;
    // test2->DFS('A');
    // cout << endl;
    // /*
    // graph* temp = test.BFS('A');
    // temp->BFS('A');*/

    // auto targets = new vector<char>;
    // targets->push_back('Z');
    // targets->push_back('F');

    // unordered_map<char, graph*> * testMap = test.parallel_aStar('A', targets);
    
    // cout << "Termina el paralelo"<< endl;

    // graph * temp = (*testMap)['Z'];
    // temp->BFS('A');
    // cout << endl;
    // temp = ((*testMap)['F'])->BFS('A');

    return EXIT_SUCCESS;
}
