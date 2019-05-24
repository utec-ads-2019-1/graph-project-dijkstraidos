#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <utility>
#include <queue>
#include <fstream>

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
        bool weighted = false;

    public:
        static self* buildFromFile(string filename){
            return new self(filename);
        }

        Graph() = default;

        Graph(string filename){
            ifstream file(filename);
            int num_of_vertices, num_of_edges;

            file>>num_of_vertices>>num_of_edges;
            file>>directed>>weighted;

            double x, y;
            N data;
            for(int i = 0; i<num_of_vertices; i++){
                file>>data>>x>>y;
                nodes.push_back(new node(data, x, y));
            }

            int l, r;
            E w;
            for(int i = 0; i<num_of_edges; i++){
                file>>l>>r;
                if(weighted){
                    file>>w;
                    addEdge(l, r, w);
                }
                else{
                    addEdge(l, r);
                }
            }
        }

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

        void addEdge(int a, int b){
            addEdge(nodes[a], nodes[b]);
        }

        void addEdge(int a, int b, E w){
            addEdge(nodes[a], nodes[b], w);
        }

        self DFS(int start){
            unordered_map<node*, int> node_to_index;
            vector<bool> vis(nodes.size(), false);
            stack<pair<int, int>> next;
            next.push(make_pair(start, -1));

            self DFSTree;
            for(int i = 0; i<nodes.size(); i++){
                DFSTree.addVertex(nodes[i]);
                node_to_index[nodes[i]] = i; 
            }

            while(!next.empty()){
                int current = next.top().first;
                int dad = next.top().second;
                next.pop();

                if(vis[current]) continue;
                vis[current] = true;
                cout<<nodes[current]->data<<endl;

                if(dad >= 0){
                    DFSTree.addEdge(dad, current);
                }

                for(edge* ep : nodes[current]->edges){
                    int vertex = node_to_index[ep->nodes[1]];
                    if(vis[vertex]) continue;
                    next.push(make_pair(vertex, current));
                }
            }

            return DFSTree;
        }

        self BFS(int start){
            unordered_map<node*, int> node_to_index;
            vector<bool> vis(nodes.size(), false);
            queue<pair<int, int>> next;
            next.push(make_pair(start, -1));

            self BFSTree;
            for(int i = 0; i<nodes.size(); i++){
                BFSTree.addVertex(nodes[i]);
                node_to_index[nodes[i]] = i; 
            }

            vis[start] = true;
            while(!next.empty()){
                int current = next.front().first;
                int dad = next.front().second;
                next.pop();
                cout<<nodes[current]->data<<endl;

                if(dad >= 0){
                    BFSTree.addEdge(dad, current);
                }

                for(edge* ep : nodes[current]->edges){
                    int vertex = node_to_index[ep->nodes[1]];
                    if(vis[vertex]) continue;
                    next.push(make_pair(vertex, current));
                    vis[vertex] = true;
                }
            }

            return BFSTree;
        }
        
};

typedef Graph<Traits> graph;

#endif
