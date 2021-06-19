#ifndef CAL_PROJ_APPLICATION_H
#define CAL_PROJ_APPLICATION_H


#include "graph/Graph.h"
#include "users/Baker.h"
#include <string>
#include <ostream>

class Application {

private:
    Graph<int> graph;
    Graph<int> invertedGraph;
    vector<Baker *> bakerSet;            // baker set
    Vertex<int>* bakery;                  // bakery vertex, origin
    int bakeryIndex;
    vector<Client *> clientSet;          // client set
    int previousTolerance;
    int afterTolerance;
    bool algorithm;                        //True if Floyd, false if Dijkstra
    bool capacityRestriction = false;

    void removeNotVisitedVertex();
    void removeClient(int clientID);
    void removeVertexDontReachBakery();
    vector<Edge<int> *> & convertNodeToEdges(vector<int> path);
    void setDeliveryPathHours();
    void multipleDeliveryDijkstraGreedy();
    void multipleDeliveryFloyd();
    void multipleDeliveryFloydGreedy();
    void multipleDeliveryDijkstra();
    void multipleDeliveryDijkstraBacktracking();
    vector<Client *> & sortedClients();
    vector<Baker>& sortedBakers();
    void dijkstraBacktracking(vector<Client * > sortedClients ,vector<Client * > currClients, long int currPathTime, long int currTime ,vector<Client * > &bestClients, long int &bestPathTime);
    void setDeliveryPathHoursWithDijkstra(Baker *baker, vector<Client * > &clients);
    void multipleBakersDeliveryDijkstraBacktracking();
    void multipleDeliveryFloydBacktracking(int bakerIndex, vector<Client*> clientVec);
    bool solveFloydBacktracking(vector<int> &test, int currClient, vector<int> &res,
                                vector<bool> canUse, long int currTime, const vector<Client *>& clientVec,const int timeToDeliver);
    void removeNotReachedFromBakeryFloyd(vector<vector<int>> &bestPaths, vector <vector<double>> &distances,
                                                      vector<double> &vec,
                                                      vector<int> &indexes);
    void removeNotReachedToBakeryFloyd(vector<vector<int>> &bestPaths, vector <vector<double>> &distances,
                                                    vector<double> &vec,
                                                    vector<int> &indexes);
    long int addFloydPathToBaker( int source, int dest, Baker* baker);
    void addClientPathToBaker(int source, Baker* baker, long int &currTime, Client* client);
    Vertex<int> * getCurrBakerVertex(Baker * baker);
    void multipleBakersDeliveryFloydBacktracking();
    void removeFromClientSet(vector<Client *> clients, vector<Client *> clientVec);
    bool solveLimitedFloydBacktracking(vector<int> &test, int currClient, vector<int> &res,
        vector<bool> canUse, long int currTime, int currQuantity, const vector<Client *> &clientVec,const int timeToDeliver);
    bool biggerQuantity(const vector<int> &vec1, const vector<int> &vec2, const vector<Client *> &clients);


public:
    Application();

    //Getters
    Graph<int> &getGraph();

    Vertex<int> *getBakery() const;

    vector<Baker *> &getBakerSet();

    int getAfterTolerance() const;

    Client * getClientInVertex(int vertexID);

    vector<int> & getClientsID() const;

    long int getFirstBakerStart();

    long int getPathTime(vector<Edge<int> *> & path);

    //Setters
    void setPreviousTolerance(int previousTolerance);

    void setAfterTolerance(int afterTolerance);

    void setBakery(Vertex<int> *bakery);

    void setBakeryIndex(int bakeryIndex);

    void setBakerSet(vector<Baker *> bakerSet);

    void setClientSet(vector<Client *> clientSet);

    void setAlgorithm(bool algorithm);

    void oneDeliveryRun();
    void multipleDeliveryRun();
    void multipleBakersDeliveryRun();
    void multipleBakersLimitedDeliveryRun();

    void preProcessGraphBFS();
    void preProcessGraphFloyd();

    std::string printPath();

    void solveDijkstraBacktrackingMultipleBakers(vector<Client * > sortedClients ,vector<Baker> currBakers, vector<Baker> &bestBakers);

    void setCapacityRestriction(bool capacityRestriction);

    void setGraph(const Graph<int> &graph);

    void setInvertedGraph(const Graph<int> &invertedGraph);

    void removeInvertedNotVisitedVertex();

};

#endif //CAL_PROJ_APPLICATION_H
