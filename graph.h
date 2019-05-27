#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <utility>
#include <queue>
#include <climits>
#include <map>

#include "minpriorityqueue.h"
#include "node.h"
#include "edge.h"

using namespace std;

class Traits {
	public:
		typedef char N;
		typedef int E;
};

template <typename Tr>
class Graph {
    public:
        typedef Graph<Tr> self;
        typedef Node<self> node;
        typedef Edge<self> edge;
        typedef MinPriorityQueue<self> min_priority_queue;
        typedef vector<node*> NodeSeq;
        typedef list<edge*> EdgeSeq;
        typedef typename Tr::N N;
        typedef typename Tr::E E;
        typedef typename NodeSeq::iterator NodeIte;
        typedef typename EdgeSeq::iterator EdgeIte;

        struct u {
            node* n;
            E key;
            struct u* parent;
            u(node* n) : n(n), key(INT_MAX), parent(nullptr) {}
        };
        typedef struct u U;

        map<node*, U*> m;
        map<pair<node*, node*> , E> weights;

    private:
        NodeSeq nodes;
        NodeIte ni;
        EdgeIte ei;
        bool directed = false;

    public:
        node* addVertex(N val){
            node* newNode = new node(val);
            nodes.push_back(newNode);
            return newNode;
        }

        node* addVertex(node* oldNode){
            node* newNode = new node(oldNode);
            nodes.push_back(newNode);
            return newNode;
        }

        void addEdge(node* a, node* b){
            if(a != nullptr && b != nullptr) {
                edge* edgeLTR = new edge(a, b);
                a->edges.emplace_back(edgeLTR);
                if(!directed){
                    edge* edgeRTL = new edge(b, a);
                    b->edges.emplace_back(edgeRTL);
                }
            }
        }

        void addEdge(node* a, node* b, E w){
            if(a != nullptr && b != nullptr) {
                edge* edgeLTR = new edge(a, b, w);
                a->edges.emplace_back(edgeLTR);
                if(!directed){
                    edge* edgeRTL = new edge(b, a, w);
                    b->edges.emplace_back(edgeRTL);
                }
            }
        }

        self DFS(node* start){
            unordered_map<node*, bool> vis;
            self DFSTree;
            stack<pair<node*, node*>> next;
            next.push(make_pair(start,nullptr));

            while(!next.empty()){
                node* current = next.top().first;
                node* dad = next.top().second;
                next.pop();

                if(vis[current]) continue;
                vis[current] = true;
                cout<<current->data<<endl;

                node* newVertex = DFSTree.addVertex(current);
                if(dad){
                    DFSTree.addEdge(dad, newVertex);
                }

                for(edge* ep : current->edges){
                    if(vis[ep->nodes[1]]) continue;
                    next.push(make_pair(ep->nodes[1], newVertex));
                }
            }

            return DFSTree;
        }

        self BFS(node* start){
            unordered_map<node*, bool> vis;
            self BFSTree;
            queue<pair<node*, node*>> next;
            next.push(make_pair(start, nullptr));

            vis[start] = true;
            while(!next.empty()){
                node* current = next.front().first;
                node* dad = next.front().second;
                next.pop();
                cout<<current->data<<endl;

                node* newVertex = BFSTree.addVertex(current);
                if(dad){
                    BFSTree.addEdge(dad, newVertex);
                }

                for(edge* ep : current->edges){
                    if(vis[ep->nodes[1]]) continue;
                    next.push(make_pair(ep->nodes[1], newVertex));
                    vis[ep->nodes[1]] = true;
                }
            }
            return BFSTree;
        }

        edge* findEdge(node* n1, node* n2) {
            for(node* n : nodes) {
                if(n == n1) return n1->edgeWith(n2);
                if(n == n2) return n2->edgeWith(n1);
            }
            throw std::string("These nodes are not part of the graph.");
        }

        E weight(node* n1, node* n2) {
            // TODO: Handle the case when it's directed
            if(weights.count({n1,n2}) == 0) {
                edge* e1 = findEdge(n2,n1);
                edge* e2 = findEdge(n1, n2);
                weights[{n1,n2}] = e1->getData();
                weights[{n2,n1}] = e2->getData();
            }
            return weights[{n1,n2}];
        }

        self MST_Prim(node* r) {
            vector<pair<node*, U*> > Q;
            map<node*,bool> inQ;
            m.clear();
            for(node* n : nodes) {
                U* x = new U(n);
                m[n] = x;
                inQ[n] = true;
                Q.push_back(make_pair(n,x));
            }
            m[r]->key = 0;
            min_priority_queue::buildMinHeap(Q);
            vector<pair<node*, U*> > V = Q;
            while(Q.size() > 0) {
                U* u = min_priority_queue::heapExtractMin(Q);
                inQ[u->n] = false;
                for(edge* e : u->n->edges) {
                    U* v = m[e->nodes[1]];
                    if(inQ[v->n] && weight(u->n,v->n) < v->key) {
                        v->parent = u;
                        v->key = weight(u->n,v->n);
                    }
                }
            }
            self MST;
            for(pair<node*,U*> p : V) {
                MST.addVertex(p.first);
            }
            for(pair<node*,U*> p : V) {
                cout << "node " << p.first << " parent " << p.second->parent << endl;
                MST.addEdge(p.first, p.second->parent ? p.second->parent->n : nullptr);
            }
            return MST;
        }
};

typedef Graph<Traits> graph;

#endif
