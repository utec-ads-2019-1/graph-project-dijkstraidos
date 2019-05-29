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
        Node(Node* node){
            this->data = node->data;
        }

        bool removeEdge(Node* nToRemove) {
            for(edge * e :edges){
                if(e->nodes[1] == nToRemove){
                    delete e;
                    edges.remove(e);
                    return true;
                }
            }
            return false;
        }

        bool removeEdge(edge * edgeToRemove){
            edges.remove(edgeToRemove);
            return true;
        }

        ~Node(){
            for(edge * e : edges){
                delete e;
            }

            edges.clear();
            delete this;
        }
};

#endif
