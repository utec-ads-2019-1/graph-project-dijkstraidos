#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file;
    cin>>file;
    graph test(file);
    test.printInfo();
    cout<<test.findEdge('a','c');
    test.addEdge('a', 'c', 3);
    cout<<test.findEdge('a','c');
    return EXIT_SUCCESS;
}
