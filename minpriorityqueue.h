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

    public:
    static int left(int i) {
        return 2*i + 1;
    }

    static int right(int i) {
        return 2*i + 2;
    }

    int parent(int i) {
        return (i-1)/2;
    }

    void heapDecreaseKey(std::vector<node*>& A, int i, E key) {
        if(key > A[i]) throw std::string("new key is larger than current key");
        A[i] = key;
        while(i > 0 && A[parent(i)] > A[i]) {
            std::swap(A[i], A[parent(i)]);
            i = parent(i);
        }
    }

    void minHeapInsert(std::vector<node*>& A, E key) {
        A.push_back(INT_MAX);
        heapDecreaseKey(A, A.size()-1, key);
    }

    static void minHeapify(std::vector<std::pair<node*,U*> >& A, int i) {
        int l = left(i);
        int r = right(i);
        int smallest;
        if(l <= A.size()-1 && A[l].second->key < A[i].second->key) smallest = l;
        else smallest = i;
        if(r <= A.size()-1 && A[r].second->key < A[smallest].second->key) smallest = r;
        if(smallest != i){
            std::swap(A[smallest],A[i]);
            minHeapify(A,smallest);
        }
    }

    static U* heapExtractMin(std::vector<std::pair<node*,U*> >& A) {
        if(A.size() < 1) throw std::string("heap underflow");
        U* min = A[0].second;
        A[0] = A[A.size()-1];
        A.pop_back();
        minHeapify(A,0);
        return min;
    }

    static void buildMinHeap(std::vector<std::pair<node*,U*> >& A) {
        int l = A.size()/2 - 1;
        for(int i = l; i >= 0; --i) minHeapify(A, i);
    }
};

#endif