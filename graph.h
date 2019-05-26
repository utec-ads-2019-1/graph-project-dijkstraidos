#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <utility>
#include <queue>
#include <algorithm>

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
        typedef Graph<Tr> Self;
        typedef Node<Self> node;
        typedef Edge<Self> edge;
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
        bool directed; //0 para not directed, 1 to Directed

    public:

        bool isDirected(){
            return directed;
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
            edge* edgeRTL = new edge(b, a);
            b->edges.emplace_back(edgeRTL);
        }


        void directed_addEdge(node* a, node* b){
            edge* newEdge = new edge(a, b);
            a->edges.emplace_back(newEdge);
        }

        void directed_addEdge(node* a, node* b, E w){
            edge* newEdge = new edge(a, b, w);
            a->edges.emplace_back(newEdge);
        }

        void addEdge(node* a, node* b, E w){
            edge* edgeLTR = new edge(a, b, w);
            a->edges.emplace_back(edgeLTR);
            edge* edgeRTL = new edge(b, a, w);
            b->edges.emplace_back(edgeRTL);
        }

        void removeVertex(node * nToRemove){
            ni = find(nodes.begin(), nodes.end(), nToRemove);
            if (ni != nodes.end()){ //si el nodo existe
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ++ei){
                    if((*ei)->nodes[0] == nToRemove){
                        (*ei)->nodes[1]->removeEdge(nToRemove);
                    }else{
                        (*ei)->nodes[0]->removeEdge(nToRemove);
                    }
                }
                nodes.erase(ni);
            }else{
                throw ("El vértice no existe");
            }
        }

        void removeEdge(node * node1, node * node2){
            node1->removeEdge(node2);
            node2->removeEdge(node1);
        }

        void directed_removeEdge(node * node1, node * node2){
            node1->removeEdge(node2);
        }

        bool findVertex(node * searched){
            if(find(nodes.begin(), nodes.end(), searched) != nodes.end())  return true;
            return false;
        }

        bool findEdge(node * node1, node * node2){
            if(!findVertex(node2)) throw ("No existe el nodo 2");

            ni = find(nodes.begin(), nodes.end(), node1);
            if(ni != nodes.end()){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ++ei){
                    if((*ei)->nodes[1] == node2) return true;
                }
                return false;
            }else throw ("No existe el nodo 1");
        }

        int nonDirected_countEdges(){
            int count = 0;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                count += (*ni)->edges.size();
            }
            return count/2;
        }

        int directed_countEdges(){
            int count = 0;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                count += (*ni)->edges.size();
            }
            return count;
        }

        bool nonDirected_isDense(float limit){
            if((2*nonDirected_countEdges())/(nodes.size()*(nodes.size()-1)) > limit){
                return true;
            }
            return false;
        }

        bool directed_isDense(float limit){
            if((directed_countEdges())/(nodes.size()*(nodes.size()-1)) > limit){
                return true;
            }
            return false;
        }


        bool nonDirected_isConnected(){
            if(nodes.size() == 0) throw ("Este grafo no tiene elementos");
            unordered_map<node*, bool> visited;
            queue<node*> next;
            next.push(nodes[0]);
            node *current;

            while(!next.empty()){
                current = next.front();
                next.pop(); //this should delete the one at the front

                for(ei = current->edges.begin(); ei != current->edges.end(); ei++){
                    if(!(visited[(*ei)->nodes[1]])) {
                        next.push((*ei)->nodes[1]);
                        visited[(*ei)->nodes[1]] = true;
                    }
                }
            }

            if(visited.size() == nodes.size()) return true;
            return false;
        }

        bool directed_isConnected(){
            if(nodes.size() == 0) throw ("Este grafo no tiene elementos");

        }


        bool isStronglyConnected(){ //solo si es un grafo no dirigido
            //TODO
        }

        /*bool isBipartite(){
            unordered_map<node*, bool> leftNodes;
            unordered_map<node*, bool> rightNodes;
            queue<node*> next;

            unordered_map<node*, bool> * currentSide;
            unordered_map<node*, bool> * otherSide;

            currentSide = &leftNodes;
            otherSide = &rightNodes;

            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                if(leftNodes[*ni] or rightNodes[*ni]) continue;

                (*currentSide)[*ni] = true;
                for(ei = (*ni).edges.begin(); ei !=(*ni).edges.begin(); ei++){
                    if((*currentSide)[(*ei)[1]]) return false;
                    otherSide[(*ei)[1]] = true;
                }

            }

            return true;
        }*/

        void printTypes(){
            //TODO
        }

        void directed_printDegree(node * n){
            int count = 0;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                    if(*ei == n) count++;
                }
            }

            cout << "El vértice " << n->data << " tiene un grado de entrada de " << count << " y un grado de salida de " << n->edges.size() << endl;
        }

        void nonDirected_printDegree(node * n){
            cout << "El vértice " << n->data << " tiene un grado de " << n->edges.size() << endl;
        }

        void printAllDegrees(){
            for (ni = nodes.begin(); ni != nodes.end(); ni++){
                nonDirected_printDegree(*ni);
            }
        }


        Self DFS(node* start){
            unordered_map<node*, bool> visited;
            Self DFSTree;
            stack<pair<node*, node*>> next;
            next.push(make_pair(start,nullptr));

            while(!next.empty()){
                node* current = next.top().first;
                node* dad = next.top().second;
                next.pop();

                if(visited[current]) continue;
                visited[current] = true;
                cout<<current->data<<endl;

                node* newVertex = DFSTree.addVertex(current);
                if(dad){
                    DFSTree.addEdge(dad, newVertex);
                }

                for(edge* ep : current->edges){
                    if(visited[ep->nodes[1]]) continue;
                    next.push(make_pair(ep->nodes[1], newVertex));
                }
            }

            return DFSTree;
        }

        Self BFS(node* start){
            unordered_map<node*, bool> visited;
            Self BFSTree;
            queue<pair<node*, node*>> next;
            next.push(make_pair(start, nullptr));

            visited[start] = true;
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
                    if(visited[ep->nodes[1]]) continue;
                    next.push(make_pair(ep->nodes[1], newVertex));
                    visited[ep->nodes[1]] = true;
                }
            }

            return BFSTree;
        }

        /*self prim_MST(node* start){
            class Comp{
                public:
                    bool operator()(edge* a, edge* b){
                        return a->data < b->data;
                    }
            };

            priority_queue<edge*, vector<edge*>, Comp> next(Comp);
            next.push(start);

            self MST;

            while(*/
            
};

typedef Graph<Traits> graph;

#endif