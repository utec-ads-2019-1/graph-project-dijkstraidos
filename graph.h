#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <list>
#include <stack>
#include <unordered_map>
#include <map>
#include <utility>
#include <queue>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>

#include "node.h"
#include "edge.h"

using namespace std;

enum vertex_Type {aislado, hoja, hundido, fuente, normal};

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

        bool nonDirected_addEdge(node* a, node* b){
            edge* edgeLTR = new edge(a, b);
            a->edges.emplace_back(edgeLTR);
            edge* edgeRTL = new edge(b, a);
            b->edges.emplace_back(edgeRTL);
            return true;
        }


        bool directed_addEdge(node* a, node* b){
            edge* newEdge = new edge(a, b);
            a->edges.emplace_back(newEdge);
            return true;
        }

        bool directed_addEdge(node* a, node* b, E w){ edge* newEdge = new edge(a, b, w);
            a->edges.emplace_back(newEdge);
            return true;
        }

        bool nonDirected_addEdge(node* a, node* b, E w){
            edge* edgeLTR = new edge(a, b, w);
            a->edges.emplace_back(edgeLTR);
            edge* edgeRTL = new edge(b, a, w);
            b->edges.emplace_back(edgeRTL);
            return true;
        }

        bool directed_removeVertex(node * nToRemove){
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                    auto k = (*ei);
                    if((*ei)->nodes[1] == nToRemove) (*ni)->edges.erase(ei);
                    break;
                }
            }
            ni = find(nodes.begin(), nodes.end(), nToRemove);
            nodes.erase(ni);
            delete nToRemove;
            return true;
        }

        bool nonDirected_removeVertex(node * nToRemove){
            for (ei = nToRemove->edges.begin(); ei != nToRemove->edges.end(); ++ei) {
                (*ei)->nodes[1]->removeEdge(nToRemove);
            }
            ni = find(nodes.begin(), nodes.end(), nToRemove);
            nodes.erase(ni);
            delete nToRemove;
            return true;
        }

        bool nonDirected_removeEdge(node * node1, node * node2){
            return node1->removeEdge(node2) and node2->removeEdge(node1);
        }

        bool directed_removeEdge(node * node1, node * node2){
            return node1->removeEdge(node2);
        }

        int absoluteEdgeCount(){
            int count = 0;
            for(ni = nodes.begin(); ni != nodes.end(); ni++){
                count += (*ni)->edges.size();
            }
            return count;
        }

        bool nonDirected_isDense(float limit){
            if((2*countEdges())/(nodes.size()*(nodes.size()-1)) > limit){
                return true;
            }
            return false;
        }

        bool directed_isDense(float limit){
            if((countEdges())/(nodes.size()*(nodes.size()-1)) > limit){
                return true;
            }
            return false;
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

        bool directed_isBipartite(){
            unordered_map<node*, bool> visited;
            stack<pair<node*, bool>> next;
            bool allMapped = false;
            for(int i = 0; i < nodes.size(); i++){
                next.push(make_pair(nodes[i], true));
                while(!next.empty()){
                    node* current = next.top().first;
                    bool color = next.top().second;
                    next.pop();

                    if(visited.find(current) == visited.end()){
                        visited[current] = color;
                    }else{
                        if(visited[current] != color) changeColors(visited);
                    }

                    for(edge* ep : current->edges){
                        if(visited.find(ep->nodes[1]) == visited.end()){
                            next.push(make_pair(ep->nodes[1], not color));
                        }else{
                            if(visited[ep->nodes[1]] == color) return false;
                        }
                    }
                }
                if(visited.size() == nodes.size()) break;
            }
            return true;
        }

        void changeColors(unordered_map<node*, bool> &myMap){
            for(auto it : myMap){
                it.second = not it.second;
            }
        }

        bool nonDirected_isBipartite(){
            unordered_map<node*, bool> visited;
            stack<pair<node*, bool>> next;
            next.push(make_pair(nodes[0], true));
            while(!next.empty()){
                node* current = next.top().first;
                bool color = next.top().second;
                next.pop();

                if(visited.find(current) == visited.end()){
                    visited[current] = color;
                }else{
                    if(visited[current] == color) return false;
                }

                for(edge* ep : current->edges){
                    if(visited.find(ep->nodes[1]) == visited.end()){
                        next.push(make_pair(ep->nodes[1], not color));
                    }else{
                        if(visited[ep->nodes[1]] == color) return false;
                    }
                }
            }
            return true;
        }


        vertex_Type nonDirected_getType(node * n){
            switch(n->edges.size()) {
                case 0:
                    return aislado;
                case 1:
                    return hoja;
                default:
                    return normal;
            }
        }

        vertex_Type directed_getType(node * n){
            int inDegree = getInDegree(n);
            if(getOutDegree(n) == 0 and inDegree == 0) return aislado;
            else if (getOutDegree(n) == 0) return hundido;
            else if (inDegree == 0) return fuente;
            else return normal;
        }

    public:
        static self* buildFromFile(string filename){
            return new self(filename);
        }

        Graph() = default;

        explicit Graph(string filename){
            ifstream file(filename);
            int num_of_vertices, num_of_edges;

            file>>num_of_vertices>>num_of_edges;
            file>>directed>>weighted;

            double x, y;
            N data;
            for(int i = 0; i<num_of_vertices; i++){
                cout<<i<<endl;
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

       explicit Graph(bool directed) : directed(directed) {};

       Graph(bool directed, bool weighted) : directed(directed), weighted(weighted) {};

        bool isDirected(){
            return directed;
        }

        bool isWeighted() {
            return weighted;
        }

        void printInfo(){
            cout << "Este grafo es ";
            if(this->directed) cout << "dirigido y ";
            else cout << "no dirigido y ";
            if(this->weighted) cout << "ponderado. ";
            else cout << "no poderado.";
            cout << "Tiene " << nodes.size() << " nodos y " << countEdges() << " aristas.\n";
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

        bool addEdge(int a, int b){
            return addEdge(nodes[a], nodes[b]);
        }

        bool addEdge(node* a, node* b){
            if(this->weighted) __throw_invalid_argument("Falta indicar un peso para esta arista (grafo es ponderado)");
            if (a == b) return false; //no se permiten lazos
            if(this->directed) return directed_addEdge(a, b);
            return nonDirected_addEdge(a, b);
        }

        bool addEdge(int a, int b, E w){
            return addEdge(nodes[a], nodes[b], w);
        }

        bool addEdge(node * a, node * b, E w){
            if(!this->weighted) __throw_invalid_argument("Este grafo no tiene pesos en sus aristas");
            if(this->directed) directed_addEdge(a, b, w);
            else nonDirected_addEdge(a, b, w);
            return true;
        }

        bool removeVertex(node * nToRemove){
            ni = find(nodes.begin(), nodes.end(), nToRemove);
            if (ni != nodes.end()){
                if(this->directed) return directed_removeVertex(nToRemove);
                return nonDirected_removeVertex(nToRemove);
            }
            return false;
        }

        bool removeEdge(int a, int b){
            return removeEdge(nodes[a], nodes[b]);
        }

        bool removeEdge(node * node1, node * node2){
            if(this->directed) return directed_removeEdge(node1, node2);
            return nonDirected_removeEdge(node1, node2);
        }

        node * findVertex(node * searched){
            auto it = find(nodes.begin(), nodes.end(), searched);
            if(it != nodes.end()) return *it;
            return nullptr;
        }


        edge * findEdge(node * node1, node * node2){ //funciona para ambos
            if(!findVertex(node2)) throw ("No existe el nodo 2");
            ni = find(nodes.begin(), nodes.end(), node1);
            if(ni != nodes.end()){
                for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ++ei){
                    if((*ei)->nodes[1] == node2) return *ei;
                }
                return nullptr;
            }else throw ("No existe el nodo 1");
        }

        //Esta parte puede ser optimizada. Lo dejo a su discreción si crear una función que cuente la cantidad absoluta
        // y de ahí llamarla y dividirla entre dos o hacer una cosa completamente diferente
        int countEdges(){
            if(this->directed) return absoluteEdgeCount();
            return absoluteEdgeCount()/2;
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

    bool isDense(float limit){
        if (this->directed) return directed_isDense(limit);
        return nonDirected_isDense(limit);
    }


    bool isConnected(){
        if(nodes.size() == 0) throw ("Este grafo no tiene elementos");
        if(this->directed) return directed_isConnected();
        return nonDirected_isConnected();
    }


    bool isStronglyConnected(){
        if(this->directed){
            if(nodes.size() == 1) return true;
            unordered_map<node*, bool> visited;
            stack<node*> next;
            next.push(nodes[0]);

            while(!next.empty()){
                node* current = next.top();
                next.pop();

                visited[current] = true;

                for(edge* ep : current->edges){
                    if(visited[ep->nodes[1]]) continue;
                    next.push(ep->nodes[1]);
                }
            }

            if(visited.size() != nodes.size()) return false;
            for(int i = 1; i < nodes.size(); i++){
                visited.clear();
                next.push(nodes[i]);
                bool found = false;
                while(!next.empty() and !found){
                    node* current = next.top();
                    next.pop();
                    visited[current] = true;

                    for(edge* ep : current->edges){
                        if(ep->nodes[1] == nodes[0]) {
                            found = true;
                            break;
                        }
                        next.push(ep->nodes[1]);
                    }
                }
                if(!found) return false;
            }
            return true;
        }
        if(this->nonDirected_isConnected()) return true;
        return false;
    }

    bool isBipartite(){
        if(this->directed) return directed_isBipartite();
        return nonDirected_isBipartite();
    }

    vertex_Type getType(int n){
        return getType(nodes[n]);
    }

    vertex_Type getType(node * n){
        if(this->directed) return directed_getType(n);
        return nonDirected_getType(n);
    }

    //quizás esto estaŕia mejor en la clase node misma
    int getOutDegree(node * n){
        return n->edges.size();
    }

    int getInDegree(node * n){
        int count = 0;
        for (ni = nodes.begin(); ni != nodes.end(); ni++) {
            for (ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++) {
                if ((*ei)->nodes[1] == n) count++;
            }
        }
        return count;
    }

    unordered_map<node*, pair<int, int>> * directed_getAllDegrees(){
        unordered_map<node*, pair<int, int>> tally; //first es lo que salen, second es el que entra
        for(ni = nodes.begin(); ni != nodes.end(); ni++){
            if(tally.find(*ni) == tally.end()) tally[*ni] = make_pair(getOutDegree(*ni), 0);
            for(ei = (*ni)->edges.begin(); ei != (*ni)->edges.end(); ei++){
                if(tally.find((*ei)->nodes[1]) == tally.end()){
                    tally[(*ei)->nodes[1]] = 1;
                }else{
                    tally[(*ei)->nodes[1]] = tally[(*ei)->nodes[1]] +1;
                }
            }
        }
        return tally;
    }

    ~Graph(){
        for(node * it : nodes){
            delete it;
        }

        nodes.clear();
    }
};

typedef Graph<Traits> graph;

#endif
