#ifndef NODE_H
#define NODE_H

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
        Node(N data, double x, double y){
            this->data = data;
            this->x = x;
            this->y = y;
        }
        Node(Node* node){
            this->data = node->data;
            this->x = node->x;
            this->y = node->y;
        }
};

#endif
