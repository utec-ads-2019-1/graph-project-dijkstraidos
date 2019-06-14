#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file;
    cin>>file;
    graph test(file);
    test.printInfo();
    graph temp = test.BFS('a');
    temp.BFS('a');
    return EXIT_SUCCESS;
}
