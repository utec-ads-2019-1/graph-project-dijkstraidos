#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <map>
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
        bool directed = false; //0 for not directed, 1 to directed
        bool weighted = false; //0 for not weighted, 1 to directed

    public:

        Graph() = default;

       Graph(bool directed) : directed(directed) {};

       Graph(bool directed, bool weighted) : directed(directed), weighted(weighted) {};

        bool isDirected(){
            return directed;
        }

        void printInfo(){
            cout << "Este grafo es ";
            if(this->directed) cout << "dirigido y ";
            else cout << "no dirigido y ";
            if(this->weighted) cout << "ponderado. ";
            else cout << "no poderado.";
            cout << "Tiene " << nodes.size() << " nodos y " << countEdges() << " aristas.\n";
            //also si es bipartito, conexo y demás
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
            if(this->weighted) __throw_invalid_argument("Falta indicar un peso para esta arista (grafo es ponderado)");
            if(this->directed) directed_addEdge(a, b);
            else nonDirected_addEdge(a, b);
        }

        void addEdge(node * a, node * b, E w){
            if(!this->weighted) __throw_invalid_argument("Este grafo no tiene pesos en sus aristas");
            if(this->directed) directed_addEdge(a, b, w);
            else nonDirected_addEdge(a, b, w);
        }

        void nonDirected_addEdge(node* a, node* b){
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

        void nonDirected_AddEdge(node* a, node* b, E w){
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
            if(this->directed) directed_removeEdge(node1, node2);
            else nonDirected_removeEdge(node1, node2);
        }

        void nonDirected_removeEdge(node * node1, node * node2){
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


        bool findEdge(node * node1, node * node2){ //funciona para ambos
            if(!findVertex(node2)) throw ("No existe el nodo 2");

            ni = find(nodes.begin(), nodes.end(), node1);
            if(ni != nodes.end()){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ++ei){
                    if((*ei)->nodes[1] == node2) return true;
                }
                return false;
            }else throw ("No existe el nodo 1");
        }

        int countEdges(){
            if(this->directed) return directed_countEdges();
            return nonDirected_countEdges();
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

        bool isDense(float limit){
            if (this->directed) return directed_isDense(limit);
            return nonDirected_isDense(limit);
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

        bool isConnected(){
            if(nodes.size() == 0) throw ("Este grafo no tiene elementos");
            if(this->directed) return directed_isConnected();
            return nonDirected_isConnected();
        }

        bool nonDirected_isConnected(){
            unordered_map<node*, bool> visited;
            queue<node*> next;
            next.push(nodes[0]);
            node *current;
            while(!next.empty()){
                current = next.front();
                next.pop();
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
            unordered_map<node*, node*> set;
            //set[nodes[0]] = nodes[0];

            /*for(int i = 0; i < nodes.size(); i++){
                set[nodes[i]] = nodes[i];
            }*/

            for(int i = 0; i < nodes.size(); i++){
                for(ei = nodes[i]->edges.begin(); ei != nodes[i]->edges.end(); ei++){
                    joinSet(set, nodes[i], (*ei)->nodes[1]);
                }
            }
            node * lastRoot = set[nodes[0]];
            for(auto it = set.begin(); it != set.end(); it++){
                if(!areInSameSet(set, (*it).second, lastRoot)) return false;
            }
            return true;
        }

        void joinSet(unordered_map<node*, node*> &set, node * a, node * b){
            set[findRoot(set, a)] = set[findRoot(set, b)];
        }

        node* findRoot(unordered_map<node*, node*> &set, node * a){
            while(a != set[a]){
                set[a] = set[set[a]]; //actualizar el valor de padre al padre de su padre
                a = set[a]; // a ahora también es el padre de su padre
            }
            return a;
        }

        bool areInSameSet(unordered_map<node*, node*> &set, node * a, node *b){
            if(findRoot(set, a) == findRoot(set, b)) return true;
            return false;
        }

/*
        void joinSet(int set[], int a, int b){
            set[find(set, a)] = find(set, b);
        }

        bool areInSameSet(int set[], int a, int b ){
            if(find(set, a) == find(set, b)) return true;
            return false;
        }

        int find(int set[], int a){
            while(a != set[a]){
                set[a] = set[set[a]]; //actualizar el valor de padre al padre de su padre
                a = set[a]; // a también es el padre de su padre
            }
        }
*/

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
            if(this->directed) directed_printTypes();
            else nonDirected_printAllTypes();
        }

        void nonDirected_printAllTypes(){
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                nonDirected_printType(*ni);
            }
        }

        void nonDirected_printType(node * n){
            switch(n->edges.size()){
                case 0:
                    cout << "El vértice " << n->data << " es un vértice aislado\n";
                    break;
                case 1:
                    cout << "El vértice " << n->data << " es un vértice hoja\n";
                    break;
                default:
                    cout << "El vértice " << n->data << " no tiene ningún tipo en especial.\n";
            }
        }

        void directed_printTypes(){
            map<node*, int> tally;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                if(tally.find(*ni) == tally.end()) tally[*ni] = 0;
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                    if(tally.find((*ei)->nodes[1]) == tally.end()){
                        tally[(*ei)->nodes[1]] = 1;
                    }else{
                        tally[(*ei)->nodes[1]] = tally[(*ei)->nodes[1]] +1;
                    }
                }
            }
            for(auto it = tally.begin(); it != tally.end(); it++){
                if(it->first->edges.size() == 0 and it->second == 0) cout << "El vértice " << it->first->data << " es un vértice aislado<\n";
                else if (it->first->edges.size() == 0)cout << "El vértice " << it->first->data << " es un hundido.\n";
                else if (it->second == 0) cout << "El vértice " << it->first->data << " es una fuente.\n";
                else cout << "El vértice " << it->first->data << " no tiene ningún tipo en especial\n";
            }
        }


        void directed_printDegree(node * n){
            int count = 0;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                    if((*ei)->nodes[1] == n) count++;
                }
            }
            cout << "El vértice " << n->data << " tiene un grado de entrada de " << count << " y un grado de salida de " << n->edges.size() << endl;
        }


        void directed_printAllDegrees(){
            map<node*, int> tally; //first es lo que salen, second es el que entra
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                if(tally.find(*ni) == tally.end()) tally[*ni] = 0;
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                    if(tally.find((*ei)->nodes[1]) == tally.end()){
                        tally[(*ei)->nodes[1]] = 1;
                    }else{
                        tally[(*ei)->nodes[1]] = tally[(*ei)->nodes[1]] +1;
                    }
                }
            }
            for(auto it = tally.begin(); it != tally.end(); it++){
                cout << "El vértice " << it->first->data << " tiene un grado de salida de " << it->first->edges.size() << " y un grado de entrada de " << it->second <<endl;
            }
        }

        void nonDirected_printDegree(node * n){
            cout << "El vértice " << n->data << " tiene un grado de " << n->edges.size() << endl;
        }

        void printDegrees(){
            if(this->directed){
                directed_printAllDegrees();
            }else{
                for (ni = nodes.begin(); ni != nodes.end(); ni++){
                    nonDirected_printDegree(*ni);
                }
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