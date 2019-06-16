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
#include <set>
#include <climits>

#include "node.h"
#include "edge.h"
#include "disjointSet.h"

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
        typedef typename Tr::N N;
        typedef typename Tr::E E;
        typedef Node<self> node;
        typedef Edge<self> edge;
        typedef unordered_map<N, node*> NodeSeq;
        typedef vector<edge*> EdgeSeq;
        typedef typename NodeSeq::iterator NodeIte;
        typedef typename EdgeSeq::iterator EdgeIte;

    private:
        NodeSeq nodes;
        NodeIte ni;
        EdgeIte ei;
        bool directed = false;
        bool weighted = false;
        
    public:

        Graph() = default;
        explicit Graph(string);
        explicit Graph(bool directed) : directed(directed) {};
        Graph(bool directed, bool weighted) : directed(directed), weighted(weighted) {};

        bool isDirected(){return directed;}
        bool isWeighted(){return weighted;}

        void addVertex(N);
        void addVertex(node*);

        bool addEdge(N, N);
        bool addEdge(N, N, E);
        bool addEdge(node*, node*, E);

        bool removeVertex(N);
        bool removeEdge(N, N);

        bool findNode(N);
        bool findEdge(N, N);
        bool findEdge(node*, node*);

        void printInfo();
        int countEdges();
        bool isDense(float limit);
        self transpose();
        vertex_Type getType(N);
        vertex_Type getType(node*);
        E graphWeight();

        int getInDegree(N);
        int getOutDegree(N);
        unordered_map<N, pair<int, int>> degrees();

        bool connected();
        bool strongly_connected();
        bool bipartite();

        self* BFS(N);
        self* DFS(N);

        unordered_map<N, pair<E, N>> dijkstra(N);
        self* kruskalMST();
        self* primMST(N);

        unordered_map<N, unordered_map<N,E>> FWSP();
        self * aStar(N, N);

        unordered_map<N, pair<double, double>> getNodesOGL(){
            unordered_map<N, pair<double, double>> pos;
            for(auto nd : nodes){
                pos[nd.first] = make_pair(nd.second->x, nd.second->y);
            }
            return pos;
        }
        vector<pair<pair<N, N>, E>> getEdgesOGL(){
            unordered_map<N, unordered_map<N, bool>> added;
            vector<pair<pair<N, N>, E>> pos;
            for(auto nd : nodes){
                for(edge* edg : nd.second->edges){
                    N adjNd = edg->nodes[1]->data;
                    if(!directed && added[adjNd][nd.first]) continue;
                    added[nd.first][adjNd] = true;
                    pos.push_back(make_pair(make_pair(nd.first, adjNd), edg->getData()));
                }
            }
            return pos;
        }

        ~Graph(){
            for(auto it : nodes){
                delete it.second;
            }

            nodes.clear();
        }

    private:

        void directed_addEdge(node* a, node* b, E w);
        void nonDirected_addEdge(node* a, node* b, E w);

        void directed_removeVertex(node * nToRemove);
        void nonDirected_removeVertex(node * nToRemove);

        bool removeEdge(node * node1, node * node2);

        int absoluteEdgeCount();

        vertex_Type nonDirected_getType(node*);
        vertex_Type directed_getType(node*);

        void updateTentativeDistance(edge *, N, N, pair<N, E> &, E &);
        
        int getManhattan(N, N);
};

template <typename Tr>
Graph<Tr>::Graph(string filename){
    ifstream file(filename);
    int num_of_vertices, num_of_edges;

    file>>num_of_vertices>>num_of_edges;
    file>>directed>>weighted;

    double x, y;
    N data;
    for(int i = 0; i<num_of_vertices; i++){
        file>>data>>x>>y;
        nodes[data] = new node(data, x, y);
    }

    N l, r;
    E w = 1;
    for(int i = 0; i<num_of_edges; i++){
        file>>l>>r;
        if(weighted){
            file>>w;
        }
        addEdge(l, r, w);
    }
}

//Add Vertex

template <typename Tr>
void Graph<Tr>::addVertex(N val){
    node* newNode = new node(val);
    nodes[val] = newNode;
}

template <typename Tr>
void Graph<Tr>::addVertex(node* oldNode){
    node* newNode = new node(oldNode);
    nodes[oldNode->data] = newNode;
}

// Add Edge

template <typename Tr>
bool Graph<Tr>::addEdge(N a, N b){
    return addEdge(a, b, 1);
}

template <typename Tr>
bool Graph<Tr>::addEdge(N a, N b, E w){
    bool find_a = nodes.find(a) == nodes.end();
    bool find_b = nodes.find(b) == nodes.end();
    if(find_a || find_b){
        throw "Not a Value on the graph!";
    }
    return addEdge(nodes[a], nodes[b], w);
}

template <typename Tr>
bool Graph<Tr>::addEdge(node * a, node * b, E w){
    if(this->directed) directed_addEdge(a, b, w);
    else nonDirected_addEdge(a, b, w);
    return true;
}

template <typename Tr>
void Graph<Tr>::directed_addEdge(node* a, node* b, E w){ edge* newEdge = new edge(a, b, w);
    a->edges.emplace_back(newEdge);
}

template <typename Tr>
void Graph<Tr>::nonDirected_addEdge(node* a, node* b, E w){
    edge* edgeLTR = new edge(a, b, w);
    a->edges.emplace_back(edgeLTR);
    edge* edgeRTL = new edge(b, a, w);
    b->edges.emplace_back(edgeRTL);
}

//Remove Vertex

template <typename Tr>
bool Graph<Tr>::removeVertex(N n){
    if(nodes.find(n) == nodes.end()) return false;
    if(this->directed){
        directed_removeVertex(nodes[n]);
    }
    else{
        nonDirected_removeVertex(nodes[n]);
    }
    delete nodes[n];
    nodes.erase(n);
    return true;
}

template <typename Tr>
void Graph<Tr>::directed_removeVertex(node * nToRemove){
    for(auto it : nodes){
        node* vert = it.second;
        vert->removeEdge(nToRemove);
    }
}

template <typename Tr>
void Graph<Tr>::nonDirected_removeVertex(node * nToRemove){
    for(edge* e : nToRemove->edges){
        node* vert = e->nodes[1];
        vert->removeEdge(nToRemove);
    }
}

//Remove Edge

template <typename Tr>
bool Graph<Tr>::removeEdge(N a, N b){
    return removeEdge(nodes[a], nodes[b]);
}

template <typename Tr>
bool Graph<Tr>::removeEdge(node * node1, node * node2){
    node1->removeEdge(node2);
    if(directed) node2->removeEdge(node1);
    return true;
}

//find functions

template <typename Tr>
bool Graph<Tr>::findNode(N val){
    return nodes.find(val) != nodes.end();
}

template <typename Tr>
bool Graph<Tr>::findEdge(N a, N b){
    return findEdge(nodes[a], nodes[b]);
}

template <typename Tr>
bool Graph<Tr>::findEdge(node *node1, node *node2){
    for(edge* edg : node1->edges){
        if(edg->nodes[1] == node2) return true;
    }
    return false;
}

//Utility Functions

template <typename Tr>
void Graph<Tr>::printInfo(){
    cout << "Este grafo es ";
    if(this->directed) cout << "dirigido y ";
    else cout << "no dirigido y ";
    if(this->weighted) cout << "ponderado. ";
    else cout << "no poderado.";
    cout << "Tiene " << nodes.size() << " nodos y " << countEdges() << " aristas.\n";
}

template <typename Tr>
int Graph<Tr>::countEdges(){
    if(this->directed) return absoluteEdgeCount();
    return absoluteEdgeCount()/2;
}

template <typename Tr>
int Graph<Tr>::absoluteEdgeCount(){
    int count = 0;
    for(auto it : nodes){
        count += it.second->edges.size();
    }
    return count;
}

template <typename Tr>
bool Graph<Tr>::isDense(float limit){
    int maxEdges = nodes.size()*(nodes.size()-1);
    int numEdges = absoluteEdgeCount();
    return numEdges/maxEdges;
}

template <typename Tr>
Graph<Tr> Graph<Tr>::transpose(){
    self TG(directed, weighted);

    for(auto it : nodes){
        TG.addVertex(it.second);
    }

    for(auto it : nodes){
        N start = it.first;
        for(edge * edg : it.second->edges){
            N finish = edg->nodes[1]->data;
            TG.addEdge(finish, start, edg->getData());
        }
    }

    return TG;
}

template <typename Tr>
vertex_Type Graph<Tr>::nonDirected_getType(node * n){
    if(!findVertex(n)) throw "Este vertice no esta en el grafo";
    switch(n->edges.size()) {
        case 0:
            return aislado;
        case 1:
            return hoja;
        default:
            return normal;
    }
}

template <typename Tr>
vertex_Type Graph<Tr>::directed_getType(node * n){
    if(!findVertex(n)) throw "Este vertice no esta en el grafo";
    int inDegree = getInDegree(n);
    if(getOutDegree(n) == 0 and inDegree == 0) return aislado;
    else if (getOutDegree(n) == 0) return hundido;
    else if (inDegree == 0) return fuente;
    else return normal;
}

template <typename Tr>
vertex_Type Graph<Tr>::getType(N n){
    return getType(nodes[n]);
}

template <typename Tr>
vertex_Type Graph<Tr>::getType(node * n){
    if(this->directed) return directed_getType(n);
    return nonDirected_getType(n);
}

template <typename Tr>
typename Graph<Tr>::E Graph<Tr>::graphWeight(){
    E totalWeight = 0;
    for(node* nd : nodes){
        for(edge* edg : nd->edges){
            totalWeight += edg->getData();
        }
    }
    if(!directed) totalWeight /= 2;
    return totalWeight;
}

template <typename Tr>
int Graph<Tr>::getOutDegree(N n){
    return nodes[n]->edges.size();
}

template <typename Tr>
int Graph<Tr>::getInDegree(N n){
    if(!directed) return getOutDegree(n);

    int edgs = 0;

    for(auto it : nodes){
        if(it.first == n) continue;
        for(edge* edg : it.second->edges){
            N adjNode = edg->nodes[1]->data;
            if(adjNode == n) edgs++;
        }
    }

    return edgs;
}

template <typename Tr>
unordered_map<typename Graph<Tr>::N, pair<int, int>> Graph<Tr>::degrees(){
    unordered_map<N, pair<int, int>> dgrs;

    for(auto it : nodes){
        dgrs[it.first].first = it.second->edges.size();
        for(edge* edg : it.second->edges){
            N nd = edg->nodes[1]->data;
            dgrs[nd].second++;
        }
    }

    return dgrs;
}

//Connectivity

template <typename Tr>
bool Graph<Tr>::connected(){
    disjoint_set<node*> ds;
    for(auto it : nodes){
        ds[it.second] = it.second;
    }

    for(auto it : nodes){
        for(edge* e : it.second->edges){
            ds.joinSet(e->nodes[0], e->nodes[1]);
        }
    }

    node* rt = ds.findRoot((*nodes.begin()).second);;
    for(auto it : nodes){
        if(ds.findRoot(it.second) != rt){
            return false;
        }
    }

    return true;
}

template <typename Tr>
bool Graph<Tr>::strongly_connected(){
    self * bfs = BFS(nodes.begin()->first);

    if(bfs->nodes.size() != nodes.size()){
        return false;
    }
    

    self trans_graph = transpose();
    self* tbfs = trans_graph.BFS(nodes.begin()->first);

    if(tbfs->nodes.size() != nodes.size()){
        return false;
    }

    return true;
}

template <typename Tr>
bool Graph<Tr>::bipartite(){
    unordered_map<node*, bool> color;
    queue<node*> next;

    for(auto it : nodes){
        node* nd = it.second;
        if(color.find(nd) != color.end()) continue;
        next.push(nd);
        color[nd] = true;

        while(!next.empty()){
            node* current = next.front();
            next.pop();
            for(edge* edg : current->edges){
                node* adjNode = edg->nodes[1];
                if(color.find(adjNode) == color.end()){
                    color[adjNode] = !color[current];
                    next.push(adjNode);
                }
                else if(color[adjNode] == color[current]){
                    return false;
                }
            }
        }
    }
    
    return true;
}

    
//Graph Search

template <typename Tr>
Graph<Tr>* Graph<Tr>::BFS(N start){
    unordered_map<N, N> parent;
    unordered_map<N, bool> vis;
    queue<N> next;

    self* BFSTree = new self(directed, weighted);

    vis[start] = true;
    parent[start] = start;
    next.push(start);

    while(!next.empty()){
        N current = next.front();
        next.pop();
        cout<<current<<endl;
        
        if(BFSTree->nodes.find(current) == BFSTree->nodes.end()){
            BFSTree->addVertex(nodes[current]);
        }

        if(parent[current] != current){
            BFSTree->addEdge(parent[current], current);
        }

        for(edge* edg : nodes[current]->edges){
            N adjNode = edg->nodes[1]->data;
            if(vis[adjNode]) continue;
            vis[adjNode] = true;
            parent[adjNode] = current;
            next.push(adjNode);
        }
    }

    return BFSTree;
}

template <typename Tr>
Graph<Tr>* Graph<Tr>::DFS(N start){
    unordered_map<N, N> parent;
    unordered_map<N, bool> vis;
    stack<N> next;

    self* DFSTree = new self(directed, weighted);

    parent[start] = start;
    next.push(start);

    while(!next.empty()){
        N current = next.top();
        next.pop();
        if(vis[current]) continue;
        vis[current] = true;
        cout<<current<<endl;
        
        if(DFSTree->nodes.find(current) == DFSTree->nodes.end()){
            DFSTree->addVertex(nodes[current]);
        }

        if(parent[current] != current){
            DFSTree->addEdge(parent[current], current);
        }

        for(edge* edg : nodes[current]->edges){
            N adjNode = edg->nodes[1]->data;
            if(vis[adjNode]) continue;
            parent[adjNode] = current;
            next.push(adjNode);
        }
    }

    return DFSTree;
}

//Minumum Spanning Tree

template <typename Tr>
Graph<Tr>* Graph<Tr>::kruskalMST(){
    disjoint_set<N> ds;

    self* MST = new self(false, weighted); 

    auto edgeCompare = [](edge* l, edge* r){return l->getData() < r->getData();};
    multiset<edge*, decltype(edgeCompare)> edges(edgeCompare);
    
    for(auto it : nodes){
        MST->addVertex(it.second);
        ds[it.first] = it.first;
        for(edge* edg : it.second->edges){
            edges.insert(edg);
        }
    }

    for(edge* edg : edges){
        N nd1 = edg->nodes[0]->data;
        N nd2 = edg->nodes[1]->data;
        if(!ds.areInSameSet(nd1, nd2)){
            ds.joinSet(nd1, nd2);
            MST->addEdge(nd1, nd2, edg->getData());
        }
    }

    return MST;
}

template <typename Tr>
Graph<Tr>* Graph<Tr>::primMST(N start){
    unordered_map<N, N> parent;
    unordered_map<N, bool> vis;
    unordered_map<N, E> weight;

    priority_queue<pair<E, N>, vector<pair<E, N>>, greater<pair<E, N>>> pq;

    self* MST = new self(false, weighted); 

    pq.push(make_pair(0, start));
    parent[start] = start;
    while(!pq.empty()){
        N curr = pq.top().second;
        E weig = pq.top().first;
        pq.pop();

        if(vis[curr]) continue;
        vis[curr] = true;

        if(MST->nodes.find(curr) == MST->nodes.end()){
            MST->addVertex(nodes[curr]);
        }
        if(parent[curr] != curr){
            MST->addEdge(parent[curr], curr, weig);
        }

        for(edge* edg : nodes[curr]->edges){
            int nd = edg->nodes[1]->data;
            int w = edg->getData();
            if(vis[nd]) continue;
            if(weight.find(nd) == weight.end() || weight[nd] < w){
                parent[nd] = curr;
                weight[nd] = w;
                pq.push(make_pair(w, nd));
            }
        }
    }

    return MST;
}

template <typename Tr>
unordered_map<typename Graph<Tr>::N, unordered_map<typename Graph<Tr>::N, typename Graph<Tr>::E>> Graph<Tr>::FWSP(){
    unordered_map<N, unordered_map<N, E>> dist;
    E inf = numeric_limits<E>::max();

    for(auto it : nodes){
        for(auto jt : nodes){
            dist[it.first][jt.first] = inf;
            if(it.first == jt.first){
                dist[it.first][jt.first] = 0;
            }
        }
    }

    for(auto it : nodes){
        for(edge* edg : it.second->edges){
            N an = edg->nodes[1]->data;
            dist[it.first][an] = edg->getData();
        }
    }

    for(auto ik : nodes){
        N k = ik.first;
        for(auto ii : nodes){
            N i = ii.first;
            for(auto ij : nodes){
                N j = ij.first;
                if(dist[i][k] < inf  && dist[k][j] < inf){
                    dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);;
                }
            }
        }
    }

    return dist;
}

template <typename Tr>
unordered_map<typename Graph<Tr>::N, pair<typename Graph<Tr>::E, typename Graph<Tr>::N>> Graph<Tr>::dijkstra(N start){
    if(!this->weighted) throw("Este grafo no es ponderado");
    unordered_map<N, pair<E, N>> distances;
    priority_queue<pair<E, N>, vector<pair<E,N>>, greater<pair<E, N>>> next;  //priority queue de pares

    distances[start] = make_pair(0, start);
    next.push(make_pair(0,start));

    while(!next.empty()){
        N current = next.top().second;
        next.pop();
        
        E distUntilNow = distances[current].first;

        for(edge* e : nodes[current]->edges){
            if(distances.find(e->nodes[1]->data) == distances.end()){
                next.push(make_pair(distUntilNow + e->getData(), e->nodes[1]->data));
                distances[e->nodes[1]->data] = make_pair(distUntilNow + e->getData(), current);
                continue;
            }
            if(distances[e->nodes[1]->data].first > e->getData() + distUntilNow){
                distances[e->nodes[1]->data] = make_pair(e->getData() + distUntilNow, current);
                next.push(make_pair(e->getData() + distUntilNow, current));
            }
        }
    }
    return distances;
}

template <typename Tr>
int Graph<Tr>::getManhattan(N start, N end){
    return abs(nodes[start]->getX() -  nodes[end]->getX()) + abs(nodes[start]->getY() - nodes[end]->getY());
}

template <typename Tr>
Graph<Tr> * Graph<Tr>::aStar(N start, N end){
    auto path = new Graph<Tr>;
    unordered_map<N, bool> visited;
    N current = start;
    pair<N, E> leastCost;
    leastCost= make_pair(start, 0);

    do{
        path->addVertex(leastCost.first);
        if (leastCost.first != start) path->addEdge(current, leastCost.first, leastCost.second);

        if(leastCost.first == end) return path;
        visited[leastCost.first] = true;
        
        current = leastCost.first;
        int tentativeDistance = numeric_limits<E>::max();

        for(edge* e : nodes[current]->edges){
            N nextNode = e->nodes[1]->data;
            if(visited[nextNode]) continue;
            //estas líneas podrían ser reemplazadas con updateTentativeDistance
            int leastTentative = e->getData() + getManhattan(current, nextNode);
            if(leastTentative < tentativeDistance){
                leastCost = make_pair(nextNode, e->getData());
                tentativeDistance = leastTentative;
            }
        }

    }while(leastCost.first != current);
    delete path;
    return nullptr;
}
template <typename Tr>
void Graph<Tr>::updateTentativeDistance(edge * e, N current, N nextNode, pair<N, E> &leastCost, E &tentativeDistance) {
    int leastTentative = e->getData() + getManhattan(current, nextNode);
    if(leastTentative < tentativeDistance){
        leastCost = make_pair(nextNode, e->getData());
        tentativeDistance = leastTentative;
    }
}

typedef Graph<Traits> graph;

#endif
