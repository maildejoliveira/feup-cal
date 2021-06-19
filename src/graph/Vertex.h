/*
 * Vertex.h
 */
#ifndef VERTEX_H_
#define VERTEX_H_

#include "Edge.h"
#include "Graph.h"
#include "MutablePriorityQueue.h"
#include "../users/Client.h"


template<class T> class Graph;
template<class T> class Edge;

template <class T>
class Vertex {
    T info;						// content of the vertex
    std::vector<Edge<T>> adj;	// outgoing edges
    double x, y;                // cords

    double dist = 0;
    Vertex<T> *path = NULL;

    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

    void addEdge(Vertex<T> *dest, long int w, const int id);
    bool removeEdgeTo(Vertex<T> *d);

public:
    Vertex(T in, double x, double y);
    T getInfo() const;
    double getDist() const;
    Vertex *getPath() const;
    vector<Edge<T>> &getAdj() ;

    bool operator<(Vertex<T> & vertex) const;

    double getX() const;

    double getY() const;

    bool isVisited() const;

    // required by MutablePriorityQueue
    friend class Graph<T>;
    friend class MutablePriorityQueue<Vertex<T>>;


};

template<class T>
Vertex<T>::Vertex(T in, double x, double y): info(in), x(x), y(y){}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
void Vertex<T>::addEdge(Vertex<T> *d, long int w, const int id) {
    adj.push_back(Edge<T>(this,d, w, id));

}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template <class T>
Vertex<T> *Vertex<T>::getPath() const {
    return this->path;
}

template<class T>
double Vertex<T>::getX() const {
    return x;
}

template<class T>
double Vertex<T>::getY() const {
    return y;
}

template<class T>
bool Vertex<T>::isVisited() const {
    return visited;
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Vertex<T>::removeEdgeTo(Vertex<T> *d) {
    auto it = this->adj.begin();
    while(it != this->adj.end()){
        if((*it).dest == d){
            adj.erase(it);
            return true;
        }
        it++;
    }
    return false;
}

template<class T>
vector<Edge<T>> &Vertex<T>::getAdj() {
    return adj;
}


#endif /* Vertex_ */
