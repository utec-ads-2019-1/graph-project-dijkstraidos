#include <iostream>

#include "graph.h"

using namespace std;

int main(int argc, char *argv[]) {
    string file;
    cin>>file;
    graph test(file);
    test.printInfo();
    cout<<test.strongly_connected();
    return EXIT_SUCCESS;
}
