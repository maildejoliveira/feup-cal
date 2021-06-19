/*
 * Graph.h
 */
#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <queue>
#include <list>
#include <limits>
#include <cmath>
#include <stack>
#include <iostream>
#include "Edge.h"
#include "MutablePriorityQueue.h"
#include "../users/Baker.h"
#include "Vertex.h"
#include "../utils/utils.h"


#define INF std::numeric_limits<long int>::max()

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

template <class T>
class Graph {
    std::vector<Vertex<T> *> vertexSet;    // vertex set
    std::vector<Edge<T> *> edgeSet;        // edge set
    long long int numberEdges;
    std::vector<std::vector<int>> bestPaths;
    std::vector<std::vector<double>> distances;
public:
    const vector<std::vector<int>> &getBestPaths() const;

    const vector<std::vector<double>> &getDistances() const;
    Vertex<T> *findVertex(const T &in) const;
    std::pair<Vertex<T> *, int> findVertexAndIndex(const T &in) const;
    bool addVertex(const T &in, const double x, const double y);
    bool removeVertex(const T &in);
    bool addEdge(const T &sourc, const T &dest, const int id);
    bool removeEdge(const T &sourc, const T &dest);
    int getNumVertex() const;
    bool vertexSetEmpty() const;
    long long int getNumberEdges() const;

    //depth first search
    void dfsVisit(Vertex<T> *v, std::vector<T> & res) const;
    std::vector<T> dfs() const;

    //breath first search
    std::vector<T> bfs(const T & source) const;

    std::vector<Vertex<T> *>& getVertexSet();
    std::vector<Edge<T> *> getEdgeSet() const;


    // Algorithms
    void dijkstraShortestPath(const T &s);
    std::vector<T> getPath(const T &origin, const T &dest) const;

    // all pairs
    void floydWarshallShortestPath();
    std::vector<T> getfloydWarshallPath(int origIndex, int destIndex) const;
    void dijkstraTowPointsShortestPath(const T &origin, const T &dest);

    void setBestPaths(const vector<std::vector<int>> &bestPaths);

    void setDistances(const vector<std::vector<double>> &distances);

};


template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template<class T>
bool Graph<T>::vertexSetEmpty() const {
    return vertexSet.empty();
}

template <class T>
std::vector<Vertex<T> *>& Graph<T>::getVertexSet() {
    return vertexSet;
}

template<class T>std::vector<Edge<T> *> Graph<T>::getEdgeSet() const {
    return edgeSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet)
        if (v->info == in)
            return v;
    return NULL;
}

template<class T>
std::pair<Vertex<T> *, int> Graph<T>::findVertexAndIndex(const T &in) const {
    for (int i = 0; i < vertexSet.size(); ++i) {
        if((vertexSet[i])->info==in)
            return make_pair(vertexSet[i], i);
    }
    return make_pair(nullptr, -1);
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in, const double x, const double y) {
    if ( findVertex(in) != NULL)
        return false;
    vertexSet.push_back(new Vertex<T>(in, x, y));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices.
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, const int id) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    long int w = ceil(distanceToTime(cartesianDistance(v1->getX(), v1->getY(), v2->getX(), v2->getY())));
    if (v1 == NULL || v2 == NULL)
        return false;
    numberEdges++;
    v1->addEdge(v2,w, id);
    return true;
}

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    Vertex<T> *v;
    if((v = findVertex(in)) == NULL) return false;
    auto it = vertexSet.begin();
    auto it2 = vertexSet.begin();
    while(it != vertexSet.end()){
        this->removeEdge((*it)->info,in);
        if(*it == v){
            it2 = it;
        }
        it++;
    }
    vertexSet.erase(it2);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    Vertex<T> *v1, *v2;
    if((v1 = findVertex(sourc)) == NULL) return false;
    if((v2 = findVertex(dest)) == NULL) return false;

    if(v1->removeEdgeTo(v2)){
        numberEdges--;
        return true;
    }
    return false;
}


/**************** Single Source Shortest Path algorithms ************/

template<class T>
void Graph<T>::dijkstraShortestPath(const T &origin) {

    for (Vertex<T>* vertex : vertexSet) {
        vertex->dist=LONG_MAX;
        vertex->path=NULL;
    }
    Vertex<T>* vertex = findVertex(origin);
    vertex->dist=0;

    MutablePriorityQueue<Vertex<T>> q;
    q.insert(vertex);

    while (!q.empty()){
        vertex=q.extractMin();
        for (Edge<T> edge : vertex->adj) {
            if(edge.dest->dist > vertex->dist + edge.weight) {
                double oldDist = edge.dest->dist;
                edge.dest->dist = vertex->dist + edge.weight;
                edge.dest->path = vertex;
                if (oldDist == LONG_MAX) {
                    q.insert(edge.dest);
                } else {
                    q.decreaseKey(edge.dest);
                }
            }
        }
    }
}

template<class T>
void Graph<T>::dijkstraTowPointsShortestPath(const T &origin, const T &dest) {

    for (Vertex<T>* vertex : vertexSet) {
        vertex->dist=LONG_MAX;
        vertex->path=NULL;
    }
    Vertex<T>* vertex = findVertex(origin);
    vertex->dist=0;

    MutablePriorityQueue<Vertex<T>> q;
    q.insert(vertex);

    while (!q.empty()){
        vertex = q.extractMin();
        if( vertex->getInfo()  == dest) return;
        for (Edge<T> edge : vertex->adj) {
            if(edge.dest->dist > vertex->dist + edge.weight) {
                double oldDist = edge.dest->dist;
                edge.dest->dist = vertex->dist + edge.weight;
                edge.dest->path = vertex;
                if (oldDist == LONG_MAX) {
                    q.insert(edge.dest);
                } else {
                    q.decreaseKey(edge.dest);
                }
            }
        }
    }
}

template<class T>
std::vector<T> Graph<T>::getPath(const T &origin, const T &dest) const{
    std::vector<T> res;

    Vertex<T>* vertex = findVertex(dest);

    std::stack<Vertex<T>*> stack;

    bool reached = false;

    while (vertex->path != NULL){
        stack.push(vertex);
        vertex=vertex->path;
        if (vertex->info==origin){
            reached=true;
            stack.push(vertex);
            break;
        }
    }

    if(reached){
        while (!stack.empty()){
            res.push_back(stack.top()->info);
            stack.pop();
        }
    }

    return res;
}


/**************** All Pairs Shortest Path  ***************/

template<class T>
void Graph<T>::floydWarshallShortestPath() {

    std::vector<std::vector<double>> dist(vertexSet.size(),std::vector<double>(vertexSet.size(), INT_MAX));
    std::vector<std::vector<int>> path(vertexSet.size(),std::vector<int>(vertexSet.size(), -1));
    for (int i = 0; i < vertexSet.size(); ++i) {
        for (int j = 0; j < vertexSet.size(); ++j) {
            if(i==j) dist[i][j]=0;
            else{
                for (Edge<T> edge : vertexSet[i]->adj) {
                    if(edge.dest->info==vertexSet[j]->info) {dist[i][j]=edge.weight; break;}
                }
            }
        }
    }

    for (int i = 0; i < vertexSet.size(); ++i) {
        for (int j = 0; j < vertexSet.size(); ++j) {
            if(dist[i][j]!=INT_MAX){
                path[i][j]=vertexSet[i]->getInfo();
            }
        }
    }
    
    for (int k = 0; k < vertexSet.size(); k++)
    {
        // Pick all vertices as source one by one
        for (int i = 0; i < vertexSet.size(); i++)
        {
            // Pick all vertices as destination for the
            // above picked source
            for (int j = 0; j < vertexSet.size(); j++)
            {
                // If vertex k is on the shortest path from
                // i to j, then update the value of dist[i][j]
                if (dist[i][k] + dist[k][j] < dist[i][j]){
                    dist[i][j] = dist[i][k] + dist[k][j];
                    path[i][j] = path[k][j];
                }
            }
        }
    }
    distances=dist;
    bestPaths = path;
}
/*
template<class T>
std::vector<T> Graph<T>::getfloydWarshallPath(const T &orig, const T &dest) const{
    std::vector<T> res;

    int origIndex, destIndex;
    for (int i = 0; i < vertexSet.size(); ++i) {
        if(vertexSet[i]->info==orig)  origIndex=i;
        if(vertexSet[i]->info==dest) destIndex=i;
    }

    std::stack<T> stack;
    stack.push(vertexSet[destIndex]->info);

    while (origIndex!=destIndex){
        destIndex=bestPaths[origIndex][destIndex];
        stack.push(vertexSet[destIndex]->info);
    }

    while (!stack.empty()){
        res.push_back(stack.top());
        stack.pop();
    }
    return res;
}*/

template<class T>
std::vector<T> Graph<T>::getfloydWarshallPath(int origIndex, int destIndex) const{
    std::vector<T> res;

    std::stack<T> stack;
    stack.push(vertexSet[destIndex]->info);

    while (origIndex!=destIndex){
        std::pair<Vertex<T> *, int> temp=findVertexAndIndex(bestPaths[origIndex][destIndex]);
        destIndex=temp.second;
        stack.push(vertexSet[destIndex]->info);
    }

    while (!stack.empty()){
        res.push_back(stack.top());
        stack.pop();
    }
    return res;
}

/*
 * Performs a depth-first search (dfs) in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T>
std::vector<T> Graph<T>::dfs() const {

    std::vector<T> res;
    for(auto elem : this->vertexSet){
        elem->visited = false;
    }
    for(auto elem : this->vertexSet){
        if(!elem->visited){
            dfsVisit(elem,res);
        }
    }
    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent not yet visited, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->visited = true;
    res.push_back(v->info);
    for(auto elem : v->adj){
        if(!elem.dest->visited){
            dfsVisit(elem.dest,res);
        }
    }
}

/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T>
std::vector<T> Graph<T>::bfs(const T & source) const {
    std::vector<T> res;
    std::queue<Vertex<T>*> foundVert;
    Vertex<T> *ver;
    for(auto elem : this->vertexSet){
        elem->visited = false;
    }

    foundVert.push(findVertex(source));
    foundVert.front()->visited = true;

    while(!foundVert.empty()){
        ver = foundVert.front();
        foundVert.pop();
        res.push_back(ver->info);
        for(auto elem : ver->adj){
            if(!elem.dest->visited){
                foundVert.push(elem.dest);
                elem.dest->visited = true;
            }
        }
    }

    return res;
}

template<class T>
void Graph<T>::setBestPaths(const vector<std::vector<int>> &bestPaths) {
    Graph::bestPaths = bestPaths;
}

template<class T>
void Graph<T>::setDistances(const vector<std::vector<double>> &distances) {
    Graph::distances = distances;
}

template<class T>
const vector<std::vector<int>> &Graph<T>::getBestPaths() const {
    return bestPaths;
}

template<class T>
const vector<std::vector<double>> &Graph<T>::getDistances() const {
    return distances;
}

template<class T>
long long int Graph<T>::getNumberEdges() const{
    return numberEdges;
}


#endif /* GRAPH_H_ */
