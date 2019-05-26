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

        void removeEdge(Node* nToRemove) {
            for(auto it = edges.begin(); it != edges.end(); ++it){
                if((*it)->nodes[0] == nToRemove or (*it)->nodes[1] == nToRemove){
                    edges.erase(it);
                    break;
                }
            }

        }

        ~Node(){
            edges.clear();
            delete this;
        }
};

#endif
