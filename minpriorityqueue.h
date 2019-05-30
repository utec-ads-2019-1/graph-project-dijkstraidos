#ifndef MINPRIORITYQUEUE_H
#define MINPRIORITYQUEUE_H
#include <algorithm>
#include <map>
#include <utility>
#include <vector>
#include <iostream>

template <typename G>
class MinPriorityQueue{
    private:
        typedef typename G::U U;
        typedef typename G::E E;
        typedef typename G::node node;
        typedef std::map<node*,int> PosMap;
        typedef MinPriorityQueue<G> self;
        std::vector<std::pair<node*,U*> > A;
        PosMap positions = {{}};
    public:

    MinPriorityQueue() : A() {}

    MinPriorityQueue(std::vector<std::pair<node*,U*> > Q) {
        this->A = Q;
        (*this).buildMinHeap();
    }

    int left(int i) {
        return 2*i + 1;
    }

    int right(int i) {
        return 2*i + 2;
    }

    int parent(int i) {
        return (i-1)/2;
    }

    void heapDecreaseKey(U* n, E key) {
        int i = positions[n->n];
        if(key > A[i].second->key) throw std::string("new key is larger than current key");
        A[i].second->key = key;
        while(i > 0 && A[parent(i)].second->key > A[i].second->key) {
            positions[A[i].first] = parent(i);
            positions[A[parent(i)].first] = i;
            std::swap(A[i], A[parent(i)]);
            i = parent(i);
        }
    }

    void minHeapInsert(E key) {
        A.push_back(INT_MAX);
        heapDecreaseKey(A, A.size()-1, key);
    }

    void minHeapify(int i) {
        int l = left(i);
        int r = right(i);
        int smallest;
        if(l <= A.size()-1 && A[l].second->key < A[i].second->key) smallest = l;
        else smallest = i;
        if(r <= A.size()-1 && A[r].second->key < A[smallest].second->key) smallest = r;
        if(smallest != i){
            positions[A[smallest].first] = i;
            positions[A[i].first] = smallest;
            std::swap(A[smallest],A[i]);
            (*this).minHeapify(smallest);
        }
    }

    U* heapExtractMin() {
        if(A.size() < 1) throw std::string("heap underflow");
        U* min = A[0].second;
        positions.erase(min->n);
        A[0] = A[A.size()-1];
        A.pop_back();
        (*this).minHeapify(0);
        return min;
    }

    std::vector<std::pair<node*,U*> > buildMinHeap() {
        int l = A.size()/2 - 1;
        for(int i = l; i >= 0; --i) (*this).minHeapify(i);
        int i = 0;
        for(std::pair<node*,U*> p : A) {
            positions[p.first] = i++;
        }
        return A;
    }

    int size() {
        return A.size();
    }

    std::vector<std::pair<node*,U*> > getData() {
        return A;
    }

};

#endif
