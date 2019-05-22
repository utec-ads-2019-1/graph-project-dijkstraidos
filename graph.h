#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <utility>
#include <queue>

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
        typedef vector<node*> NodeSeq;
        typedef list<edge*> EdgeSeq;
        typedef typename Tr::N N;
        typedef typename Tr::E E;
        typedef typename NodeSeq::iterator NodeIte;
        typedef typename EdgeSeq::iterator EdgeIte;

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
            edge* edgeLTR = new edge(a, b);
            a->edges.emplace_back(edgeLTR);
            if(!directed){
                edge* edgeRTL = new edge(b, a);
                b->edges.emplace_back(edgeRTL);
            }
        }

        void addEdge(node* a, node* b, E w){
            edge* edgeLTR = new edge(a, b, w);
            a->edges.emplace_back(edgeLTR);
            if(!directed){
                edge* edgeRTL = new edge(b, a, w);
                b->edges.emplace_back(edgeRTL);
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

};

typedef Graph<Traits> graph;

#endif
