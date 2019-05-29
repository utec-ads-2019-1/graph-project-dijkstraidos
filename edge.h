#ifndef EDGE_H
#define EDGE_H

#include "node.h"

template <typename G>
class Edge {
    public:
        typedef typename G::E E;
        typedef typename G::node node;

        node* nodes[2];

    private:
        E data;
        bool dir;

    public:
        Edge(node* from, node* to){
            nodes[0] = from;
            nodes[1] = to;
        }

        Edge(node* from, node* to, E w){
            nodes[0] = from;
            nodes[1] = to;
            data = w;
        }

        E getData(){ 
           return data;
        }

        /*~Edge(){
            delete this;
        }*/
};

#endif
