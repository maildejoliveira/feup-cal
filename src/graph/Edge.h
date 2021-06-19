/*
 * Edge.h
 */
#ifndef EDGE_H_
#define EDGE_H_

#include "Graph.h"
#include "Vertex.h"

template <class T>
class Edge {
    int id;
    Vertex<T> * orig;      // origin vertex
    Vertex<T> * dest;      // destination vertex
    long int weight;         // edge weight
public:
    Edge(Vertex<T> *o,Vertex<T> *d, long int w, int id);
    friend class Graph<T>;
    friend class Vertex<T>;

    int getId() const;

    long int getWeight() const;

    Vertex<T> *getOrig() const;

    Vertex<T> *getDest() const;

};

template <class T>
Edge<T>::Edge(Vertex<T> *o, Vertex<T> *d, long int w, int id): orig(o),dest(d), weight(w), id(id) {}

template<class T>
long int Edge<T>::getWeight() const {
    return weight;
}

template<class T>
Vertex<T> *Edge<T>::getOrig() const {
    return orig;
}

template<class T>
Vertex<T> *Edge<T>::getDest() const {
    return dest;
}

template<class T>
int Edge<T>::getId() const {
    return id;
}

#endif /* Edge_ */
