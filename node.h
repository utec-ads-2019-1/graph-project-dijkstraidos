#ifndef NODE_H
#define NODE_H
#include <string>
#include <iostream>

template <typename G>
class Node {
    public:
        typedef typename G::N N;
        typedef typename G::E E;
        typedef typename G::edge edge;
        typedef typename G::EdgeSeq EdgeSeq;

        EdgeSeq edges;
        N data;

    private:
        double x;
        double y;

    public:
        Node(N data){
            this->data = data;
        }
        Node(Node* node){
            this->data = node->data;
        }

        N getData() {
            return data;
        }

        edge* edgeWith(Node* node) {
            for(edge* e : edges) {
                if(e->nodes[1] == node) return e;
            }
            std::cout << std::string("There is no such edge.");
        }
};

#endif
