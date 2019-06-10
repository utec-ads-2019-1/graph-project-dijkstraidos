#ifndef DSET
#define DSET

#include <unordered_map>

using namespace std;

template <typename T>
class disjoint_set{

    private:
        unordered_map<T, T> ds;

    public:

        disjoint_set() = default;

        T& operator[](T i){
            return ds[i];
        }

        void joinSet(T a, T b){
            ds[findRoot(a)] = ds[findRoot(b)];
        }

        T findRoot(T a){
            while(a != ds[a]){
                ds[a] = ds[ds[a]];
                a = ds[a];
            }
            return a;
        }

        bool areInSameSet(T a, T b){
            if(findRoot(a) == findRoot(b)) return true;
            return false;
        }
};

#endif
