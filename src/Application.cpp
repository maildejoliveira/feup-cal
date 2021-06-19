#include <algorithm>
#include <chrono>
#include "Application.h"
#include "./utils/Csvfile.h"
#include <iostream>
#include <zconf.h>

Application::Application() {
    bakerSet = vector<Baker *>();
    clientSet=vector<Client *>();
}

Graph<int> &Application::getGraph() {
    return graph;
}

void Application::setPreviousTolerance(int previousTolerance) {
    this->previousTolerance = previousTolerance;
}

void Application::setAfterTolerance(int afterTolerance) {
    this->afterTolerance = afterTolerance;
}

void Application::setBakery(Vertex<int> *bakeryVal) {
   this->bakery = bakeryVal;
}

void Application::setBakerSet(vector<Baker *> bakerSet) {
    this->bakerSet = bakerSet;
}

void Application::setClientSet(vector<Client *> clientSet) {
    this->clientSet = clientSet;
}


void Application::preProcessGraphBFS() {
    auto start = std::chrono::high_resolution_clock::now();
    long int vertexSize = graph.getVertexSet().size();
    long int edgesSize = graph.getNumberEdges();

    graph.bfs(bakery->getInfo());
    removeNotVisitedVertex();

    invertedGraph.bfs(bakery->getInfo());
    removeInvertedNotVisitedVertex();

    auto finish = std::chrono::high_resolution_clock::now();
    auto time =  chrono::duration_cast<chrono::milliseconds>(finish - start).count();
    Csvfile file(DATA_FILE_PATH);
    file << "BFS" << vertexSize << edgesSize << time;
    file.endrow();
}

void Application::removeInvertedNotVisitedVertex(){
    bool endReach = false;
    while(!endReach && !invertedGraph.getVertexSet().empty()){
        for(int i = 0 ; i < invertedGraph.getVertexSet().size(); i++){
            Vertex<int> * v = invertedGraph.getVertexSet().at(i);
            if(!v->isVisited()) {
                invertedGraph.removeVertex(v->getInfo());
                graph.removeVertex(v->getInfo());
                break;
            }

            if(i == invertedGraph.getVertexSet().size()-1)
                endReach = true;
        }
    }
}

void Application::removeNotVisitedVertex() {
    bool endReach = false;
    while(!endReach && !graph.getVertexSet().empty()){
        for(int i = 0 ; i < graph.getVertexSet().size(); i++){
            Vertex<int> * v = graph.getVertexSet().at(i);
            if(!v->isVisited()) {
                graph.removeVertex(v->getInfo());
                invertedGraph.removeVertex(v->getInfo());
                break;
            }

            if(i == graph.getVertexSet().size()-1)
                endReach = true;
        }
    }
}

Client *Application::getClientInVertex(int vertexID) {
    for(int i = 0; i < clientSet.size(); i++){
        if(clientSet.at(i)->getVertex()->getInfo() == vertexID){
            return clientSet.at(i);
        }
    }
    return nullptr;
}

void Application::removeClient(int clientID) {
    for(auto it = clientSet.begin(); it != clientSet.end(); it++){
        if((*it)->getId() == clientID) {
            clientSet.erase(it);
            return;
        }
    }
}


Vertex<int> *Application::getBakery() const {
    return bakery;
}

vector<int>& Application::getClientsID() const {
    vector<int>* clientsID = new vector<int>;
    for(int i=0; i<clientSet.size(); i++){
        clientsID->push_back(clientSet.at(i)->getVertex()->getInfo());
    }
    return *clientsID;
}

vector<Edge<int> *>& Application::convertNodeToEdges(vector<int> path){
    vector<Edge<int> *>* res = new vector<Edge<int>*>;
    for(int i = 1; i < path.size(); i++){
        Vertex<int>* v = graph.findVertex(path.at(i - 1));
        for(int j = 0; j < v->getAdj().size() ; j++ ){
            if(v->getAdj().at(j).getDest()->getInfo() == path.at(i)){
                res->push_back(& v->getAdj().at(j));
                break;
            }
        }
    }
    return *res;
}

long int Application::getFirstBakerStart(){
    long int firstStart = INF;

    for(Baker * baker : bakerSet){
        if(baker->getStartTime() < firstStart) firstStart = baker->getStartTime();
    }

    return firstStart;

}

int Application::getAfterTolerance() const {
    return afterTolerance;
}

long int Application::getPathTime(vector<Edge<int> *> & path) {
    long int pathTime = 0;
    for(Edge<int>* e: path){
        pathTime+=e->getWeight();
    }
    return pathTime;
}

vector<Baker *> &Application::getBakerSet(){
    return bakerSet;
}

void Application::setDeliveryPathHours() {
    bool firstClient=true;
    long int pathTimeSum=0;
    for(Baker* baker: bakerSet){
        pathTimeSum=0;
        for(Edge<int>* edge: baker->getPath()){
            Client* c = getClientInVertex(edge->getOrig()->getInfo());
            if(c!= nullptr){
                if(!c->isDelivered()) {
                    if (firstClient) {
                        baker->setActualStartTime(max(baker->getStartTime(), c->getScheduledTime() - pathTimeSum));
                        firstClient = false;
                    }
                    c->setActualDeliveryTime(baker->getActualStartTime() + pathTimeSum);
                    pathTimeSum += baker->getDeliveryTime();
                    c->setDelivered(true);
                }
                baker->addAttendedClient(c);
            }
            pathTimeSum+=edge->getWeight();
        }
        baker->setPathDuration(pathTimeSum);
    }
}

/**
 * In order to make this print each baker must have the following parameters defined:
 * clientsAttended
 * pathDuration
 * path
 * actualStartTime
 * @return - string with the path output
 */
std::string Application::printPath() {
    std::ostringstream path;

    for(Baker* baker: bakerSet){
        auto it = baker->getClientsAttended().begin();
        path << endl << endl;
        path << "Baker " << baker->getId() << " path has a duration of " << secondsToTime(baker->getPathDuration()) << endl;
        if(capacityRestriction) path << "Need a total of " << baker->getBreadsDelivered() << " breads for the delivery." << endl;
        path << "Left at " << secondsToTime(baker->getActualStartTime()) << " from Vertex " << bakery->getInfo() << "." << endl;
        for(Edge<int>* edge: baker->getPath()){
            path << "Edge " << edge->getId() << endl;
            path << "Vertex " << edge->getDest()->getInfo();
            Client* c = getClientInVertex(edge->getDest()->getInfo());
            if(c != nullptr && c->getId() == (*it)->getId()){
                path << " has Client " << c->getId() << " and arrived at " << secondsToTime(c->getActualDeliveryTime()) << "." << endl;
                if(capacityRestriction) path << "Baker delivered " << c->getBreadsQuantity() << " breads.";
                it = baker->getClientsAttended().erase(it);
            }
            path << endl;
        }
        path << "Arrived at the Bakery at " << secondsToTime(baker->getActualStartTime()+baker->getPathDuration()) << "." << endl;
    }
    return path.str();
}

//Problems
void Application::oneDeliveryRun() {
    Csvfile file(DATA_FILE_PATH);
    auto start = std::chrono::high_resolution_clock::now();


    vector<int> path;
    graph.dijkstraShortestPath(bakery->getInfo());
    if(!clientSet.empty()) {
        path = graph.getPath(bakery->getInfo(), clientSet.at(0)->getVertex()->getInfo());
    }
    vector<int> returnPath;
    if(!clientSet.empty()) {
        graph.dijkstraShortestPath(clientSet.at(0)->getVertex()->getInfo());
        returnPath = graph.getPath(clientSet.at(0)->getVertex()->getInfo(), bakery->getInfo());
    }
    path.insert(path.end(),returnPath.begin(),returnPath.end());

    bakerSet.at(0)->setPath(convertNodeToEdges(path));
    setDeliveryPathHours();

    auto finish = std::chrono::high_resolution_clock::now();
    auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
    file << "Dijkstra One" << graph.getVertexSet().size() << graph.getNumberEdges() << time
         << 1 << 1;
    file.endrow();
}

void Application::multipleDeliveryRun() {
    if (algorithm) {
        multipleDeliveryFloyd();
        return;
    }

    multipleDeliveryDijkstra();
}

void Application::multipleDeliveryDijkstra() {
    int i=0;
    Csvfile file(DATA_FILE_PATH);
    do {
        cout << "What methodology would you like to use?" << endl <<
             "\t1. Greedy" << endl <<
             "\t2. Backtracking" << endl;

        cin >> i;
    } while (i!=1 && i!=2);

    if(i == 1) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleDeliveryDijkstraGreedy();
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Dijkstra Greedy SB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << 1;
    }
    else if(i == 2) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleDeliveryDijkstraBacktracking();
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Dijkstra Backtracking SB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << 1;
    }
    file.endrow();
}

void Application::multipleDeliveryFloyd() {
    int i=0;
    Csvfile file(DATA_FILE_PATH);
    do {
        cout << "What methodology would you like to implement?" << endl <<
             "\t1. Greedy" << endl <<
             "\t2. Backtracking" << endl;

        cin >> i;
    } while (i!=1 && i!=2);

    if(i == 1) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleDeliveryFloydGreedy();
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Floyd Greedy SB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << 1;
    }
    if(i == 2) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleDeliveryFloydBacktracking(0, clientSet);
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Floyd Backtracking SB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << 1;
    }
    file.endrow();

}

void Application::multipleDeliveryDijkstraGreedy(){
    vector<int> path;
    vector<Edge<int> * > temp, convPath;
    vector<Client * > clients = sortedClients();
    int firstPathTime = 0;

    Vertex<int> * currVert = bakery;
    Baker * baker = bakerSet.at(0);
    long int currTime = baker->getStartTime();
    int clientsAttended = 0;

    auto it = clients.begin();
    while(!clients.empty()){
        int pathTime;
        graph.dijkstraTowPointsShortestPath(currVert->getInfo(),(*it)->getVertex()->getInfo());

        path = graph.getPath(currVert->getInfo(),(*it)->getVertex()->getInfo());
        convPath = convertNodeToEdges(path);
        pathTime = getPathTime(convPath);

        if(currTime + pathTime <= (*it)->getScheduledTime() + afterTolerance){
            clientsAttended++;
            currTime = max(currTime + pathTime, (*it)->getScheduledTime() - previousTolerance);
            (*it)->setActualDeliveryTime(currTime);
            baker->addAttendedClient(*it);
            currTime += baker->getDeliveryTime();
            currVert = (*it)->getVertex();
            temp = baker->getPath();
            temp.insert(temp.end(),convPath.begin(),convPath.end());
            baker->setPath(temp);
            if(firstPathTime == 0){
                firstPathTime = pathTime;
            }
        }
        it = clients.erase(it);
    }

    graph.dijkstraTowPointsShortestPath(currVert->getInfo(),bakery->getInfo());
    path = graph.getPath(currVert->getInfo(),bakery->getInfo());
    convPath = convertNodeToEdges(path);

    temp = baker->getPath();
    temp.insert(temp.end(),convPath.begin(),convPath.end());
    baker->setPath(temp);
    if(!baker->getClientsAttended().empty()){
        baker->setActualStartTime(baker->getClientsAttended().at(0)->getActualDeliveryTime() - firstPathTime);
        baker->setPathDuration(currTime + getPathTime(convPath) - baker->getActualStartTime());
    }
}

bool clientComparator(Client * i1, Client * i2){
    return (i1->getScheduledTime() < i2->getScheduledTime());
}

vector<Client *> &Application::sortedClients() {
    sort(clientSet.begin(),clientSet.end(), clientComparator);
    vector<Client *>* ret = new vector<Client *>(clientSet);
    return *ret;
}

bool bakerComparator(Baker * i1, Baker * i2){
    return (i1->getStartTime() < i2->getStartTime());
}
vector<Baker>& Application::sortedBakers(){
    vector<Baker> * bakers = new vector<Baker>;
    sort(bakerSet.begin(),bakerSet.end(), bakerComparator);
    for(auto & i : bakerSet){
        bakers->push_back(*i);
    }
    return *bakers;
}

void Application::multipleDeliveryFloydGreedy() {
    vector<int> path;
    vector<Edge<int> * > temp, convPath;
    vector<Client * > clients = sortedClients();
    int firstPathTime = 0;

    int currVert = bakeryIndex;
    Baker * baker = bakerSet.at(0);
    long int currTime = baker->getStartTime();
    int clientsAttended = 0;

    auto it = clients.begin();
    while(!clients.empty()){
        int pathTime;

        path = graph.getfloydWarshallPath(currVert,(*it)->getIndex());

        convPath = convertNodeToEdges(path);
        pathTime = getPathTime(convPath);

        if(currTime + pathTime <= (*it)->getScheduledTime() + afterTolerance){
            clientsAttended++;
            currTime = max(currTime + pathTime, (*it)->getScheduledTime() - previousTolerance);
            (*it)->setActualDeliveryTime(currTime);
            baker->addAttendedClient(*it);
            currTime += baker->getDeliveryTime();
            currVert = (*it)->getIndex();
            temp = baker->getPath();
            temp.insert(temp.end(),convPath.begin(),convPath.end());
            baker->setPath(temp);
            if(firstPathTime == 0){
                firstPathTime = pathTime;
            }
        }
        it = clients.erase(it);
    }

    path = graph.getfloydWarshallPath(currVert ,bakeryIndex);
    convPath = convertNodeToEdges(path);

    temp = baker->getPath();
    temp.insert(temp.end(),convPath.begin(),convPath.end());
    baker->setPath(temp);
    if(!baker->getClientsAttended().empty()){
        baker->setActualStartTime(baker->getClientsAttended().at(0)->getActualDeliveryTime() - firstPathTime);
        baker->setPathDuration(currTime + getPathTime(convPath) - baker->getActualStartTime());
    }
    //setDeliveryPathHours();

}

void Application::setBakeryIndex(int bakeryIndex) {
    Application::bakeryIndex = bakeryIndex;
}

void Application::multipleDeliveryDijkstraBacktracking() {
    vector<Client *> bestClients, currClients, clients(sortedClients());
    long int pathTime = INF;

    //remove clients that can't be reached when leaving bakery
    auto it = clients.begin();
    while (it != clients.end()) {
        long int time = getPathTime(convertNodeToEdges(graph.getPath(bakery->getInfo(),(*it)->getVertex()->getInfo())));
        if(bakerSet.at(0)->getStartTime() + time > (*it)->getScheduledTime() + afterTolerance) {
            it = clients.erase(it);
        } else {
            break;
        }
    }

    dijkstraBacktracking(clients, currClients, 0, 0, bestClients, pathTime);

    setDeliveryPathHoursWithDijkstra(bakerSet.at(0),bestClients);
}

void Application::dijkstraBacktracking(vector<Client * > sortedClients ,vector<Client * > currClients, long int currPathTime, long int currTime ,vector<Client * > &bestClients, long int &bestPathTime){
    if(sortedClients.empty()){
        if(currClients.size() > bestClients.size()) {
            bestClients = currClients;
            bestPathTime = currPathTime;
        }else if(currClients.size() == bestClients.size() && currPathTime < bestPathTime){
                bestClients = currClients;
                bestPathTime = currPathTime;
            }
        return;
    }

    vector<int> path;
    vector<Edge<int> * >convPath;
    Vertex<int> * currVert;
    long int tempTime;

    //set up begin time
    Baker * baker = bakerSet.at(0);
    if(currClients.empty()) {
        currVert = bakery;
        currTime = baker->getStartTime();
    }else{
        currVert = currClients.at(currClients.size() -1)->getVertex();
    }

    Client * nextClient = sortedClients.at(0);
    sortedClients.erase(sortedClients.begin());
    dijkstraBacktracking(sortedClients,currClients,currPathTime, currTime,bestClients,bestPathTime);

    currClients.push_back(nextClient);

    graph.dijkstraTowPointsShortestPath(currVert->getInfo(),nextClient->getVertex()->getInfo());
    path = graph.getPath(currVert->getInfo(),nextClient->getVertex()->getInfo());
    convPath = convertNodeToEdges(path);

    //verify time that it arrives to the next client
    tempTime = currTime;
    currTime -= currPathTime;
    currPathTime += getPathTime(convPath);
    currTime = max(currTime + currPathTime, nextClient->getScheduledTime() - previousTolerance);
    currPathTime += currTime - tempTime;
    currPathTime += baker->getDeliveryTime();
    currTime += baker->getDeliveryTime();


    auto it = sortedClients.begin();
    while (it != sortedClients.end()) {
        int pathTime;
        path = graph.getPath(currVert->getInfo(),(*it)->getVertex()->getInfo());
        convPath = convertNodeToEdges(path);
        pathTime = getPathTime(convPath);

        if(currTime + pathTime > (*it)->getScheduledTime() + afterTolerance) {
            it = sortedClients.erase(it);
        } else {
            it++;
        }
    }
    if(sortedClients.size() + currClients.size() < bestClients.size()) return;

    dijkstraBacktracking(sortedClients,currClients,currPathTime, currTime,bestClients,bestPathTime);
}

void Application::multipleDeliveryFloydBacktracking(int bakerIndex, vector<Client*> clientVec) {
    //Index and Edge path
    vector<int> path;
    vector<Edge<int> * > convPath;

    //Initial vertexIndex
    int currVert = bakeryIndex;
    Baker * baker = bakerSet.at(bakerIndex);
    long int currTime;
    int breadQuantity;
    int deliveryTime = baker->getDeliveryTime();

    //Vertex path res and test vectors
    vector<int> res;
    vector<int> test;
    res.push_back(bakery->getInfo());

    vector<bool> canUse(clientVec.size(),true);
    for (int i = 0; i < clientVec.size(); ++i) {
        //Reset test, currentTime
        test={};
        test.push_back(bakery->getInfo());
        currTime = baker->getStartTime();
        breadQuantity = clientVec[i]->getBreadsQuantity();

        //Find current time after going to first client
        path = graph.getfloydWarshallPath(currVert,clientVec[i]->getIndex());
        convPath = convertNodeToEdges(path);
        long int pathTime = getPathTime(convPath);

        //Can not attend client due to time distance
        if (currTime + pathTime > clientVec[i]->getScheduledTime() + afterTolerance) {
            canUse[i]=false;
            continue;
        }
        //Can not attend client due to bigger amount of bread to deliver
        if(capacityRestriction && breadQuantity > baker->getCapacity()) {
            canUse[i]=false;
            continue;
        }
        currTime = max(currTime + pathTime, clientVec[i]->getScheduledTime() - previousTolerance);

        canUse[i]= false;
        test.push_back(clientVec[i]->getVertex()->getInfo());

        if (capacityRestriction)
            solveLimitedFloydBacktracking(test, i,res, canUse, currTime+deliveryTime,
                                          baker->getCapacity()-breadQuantity,clientVec, deliveryTime);
        else
            solveFloydBacktracking(test, i,res, canUse, currTime+baker->getDeliveryTime(), clientVec,
                                   deliveryTime);
        canUse[i]= true;
    }

    currVert = bakeryIndex;
    currTime = baker->getStartTime();
    int j=1;

    //Find real path with a list of clients
    while (baker->getClientsAttended().size()!=res.size()-1){
        for (int i = 0; i < clientSet.size(); ++i) {
            Client * client = clientSet[i];
            if(client->getVertex()->getInfo()!=res[j]) continue;

            //Find path from node to node, add to Baker path
            //and add client to attended clients
            addClientPathToBaker( currVert, baker, currTime, client);
            currVert = client->getIndex();

            break;
        }
        j++;
    }

    //Add return trip to bakery
    addFloydPathToBaker( currVert, bakeryIndex, baker);


    if(!baker->getClientsAttended().empty()){
        baker->setPathDuration(currTime + getPathTime(convPath) - baker->getActualStartTime());
    }

}
bool Application::solveFloydBacktracking(vector<int> &test, int currClient, vector<int> &res,
                                         vector<bool> canUse, long int currTime,
                                         const vector<Client *> &clientVec,const int timeToDeliver) {
    vector<int> path;
    vector<Edge<int> * > convPath;
    long int pathTime;
    int available=0;

    Vertex<int>* onGoing = clientVec[currClient]->getVertex();
    for (int i = 0; i < clientVec.size(); ++i) {
        if (canUse[i]){
            //Find time to next client
            path = graph.getfloydWarshallPath(clientVec[currClient]->getIndex(),clientVec[i]->getIndex());
            convPath = convertNodeToEdges(path);
            pathTime = getPathTime(convPath);

            //If baker can reach the client on time
            if(currTime + pathTime > clientVec[i]->getScheduledTime() + afterTolerance){
                canUse[i]= false;
            }
        }
        if(canUse[i]) {available++;}
    }

    //No more clients
    if(available==0) {
        if(test.size()>res.size()) res=vector<int>(test);
        return true;
    }

    //Keep going
    for (int i = 0; i < clientVec.size(); ++i) {
        if(canUse[i]){
            //Insert new client on test
            test.push_back(clientVec[i]->getVertex()->getInfo());
            canUse[i] = false;

            //Get new currentTime
            path = graph.getfloydWarshallPath(clientVec[currClient]->getIndex(),clientVec[i]->getIndex());
            convPath = convertNodeToEdges(path);
            pathTime = getPathTime(convPath);
            long int tmpTime = max(currTime + pathTime, clientVec[i]->getScheduledTime() - previousTolerance);

            solveFloydBacktracking(test, i, res, canUse, tmpTime+timeToDeliver, clientVec, timeToDeliver);
            canUse[i]= true;
            test.pop_back();
        }
    }
    return true;
}

void Application::setDeliveryPathHoursWithDijkstra(Baker *baker, vector<Client * > &clients){
    vector<int> path;
    vector<Edge<int> * > temp, convPath;
    int firstPathTime = 0;

    Vertex<int> * currVert = bakery;
    long int currTime = baker->getStartTime();

    auto it = clients.begin();
    while(!clients.empty()){
        int pathTime;
        graph.dijkstraTowPointsShortestPath(currVert->getInfo(),(*it)->getVertex()->getInfo());

        path = graph.getPath(currVert->getInfo(),(*it)->getVertex()->getInfo());
        convPath = convertNodeToEdges(path);
        pathTime = getPathTime(convPath);

        if(firstPathTime == 0){
            firstPathTime = pathTime;
        }

        currTime = max(currTime + pathTime, (*it)->getScheduledTime() - previousTolerance);
        (*it)->setActualDeliveryTime(currTime);
        baker->addAttendedClient(*it);
        baker->addBreadsDelivered((*it)->getBreadsQuantity());
        currTime += baker->getDeliveryTime();
        currVert = (*it)->getVertex();
        temp = baker->getPath();
        temp.insert(temp.end(),convPath.begin(),convPath.end());
        baker->setPath(temp);

        it = clients.erase(it);
    }

    graph.dijkstraTowPointsShortestPath(currVert->getInfo(),bakery->getInfo());
    path = graph.getPath(currVert->getInfo(),bakery->getInfo());
    convPath = convertNodeToEdges(path);

    temp = baker->getPath();
    temp.insert(temp.end(),convPath.begin(),convPath.end());
    baker->setPath(temp);

    if(!baker->getClientsAttended().empty()){
        baker->setActualStartTime(baker->getClientsAttended().at(0)->getActualDeliveryTime() - firstPathTime); // add first delivery time
        baker->setPathDuration(currTime + getPathTime(convPath) - baker->getActualStartTime());
    }
}

void Application::preProcessGraphFloyd() {
    auto start = std::chrono::high_resolution_clock::now();
    long int vertexSize = graph.getVertexSet().size();
    long int edgeSize = graph.getNumberEdges();

    graph.floydWarshallShortestPath();
    vector<vector<int>> bestPaths=graph.getBestPaths();
    vector<vector<double>> distances=graph.getDistances();

    vector<double> vec = distances[bakeryIndex];
    vector<int> indexes;

    removeNotReachedFromBakeryFloyd(bestPaths, distances, vec, indexes);

    removeNotReachedToBakeryFloyd(bestPaths, distances, vec, indexes);
    graph.setDistances(distances);
    graph.setBestPaths(bestPaths);

    auto finish = std::chrono::high_resolution_clock::now();
    auto time =  chrono::duration_cast<chrono::milliseconds>(finish - start).count();
    Csvfile file(DATA_FILE_PATH);
    file << "Floyd-pre" << vertexSize << edgeSize << time;
    file.endrow();
}

void Application::setAlgorithm(bool algorithm) {
    Application::algorithm = algorithm;
}

void Application::removeNotReachedFromBakeryFloyd(vector<vector<int>> &bestPaths, vector <vector<double>> &distances,
                                                  vector<double> &vec,
                                                  vector<int> &indexes) {

    //Remove not reached vertex from bakery
    for (int j = vec.size()-1; j >= 0; --j) {
        if(vec[j]!=INT_MAX) continue;
        indexes.push_back(j);
        graph.removeVertex(graph.getVertexSet()[j]->getInfo());
        if(j<bakeryIndex) bakeryIndex--;
    }

    for (int i = 0; i < indexes.size(); ++i) {
        distances.erase(distances.begin()+indexes[i]);
        bestPaths.erase(bestPaths.begin()+indexes[i]);

        int size=distances.size();
        for (int j = size-1; j >= 0; --j) {
            distances[j].erase(distances[j].begin()+indexes[i]);
            bestPaths[j].erase(bestPaths[j].begin()+indexes[i]);
        }
    }
}

void Application::removeNotReachedToBakeryFloyd(vector<vector<int>> &bestPaths, vector <vector<double>> &distances,
                                                vector<double> &vec,
                                                vector<int> &indexes) {
    //Remove not reached vertex going to bakery
    indexes={};
    int decrement=0;
    for (int j = distances.size()-1; j >= 0; --j) {
        if(distances[j][bakeryIndex]!=INT_MAX) continue;
        indexes.push_back(j);
        graph.removeVertex(graph.getVertexSet()[j]->getInfo());
        if(j<(bakeryIndex-decrement)) decrement++;
    }
    bakeryIndex-=decrement;
    for (int i = 0; i < indexes.size(); ++i) {
        distances.erase(distances.begin()+indexes[i]);
        bestPaths.erase(bestPaths.begin()+indexes[i]);

        int size=distances.size();
        for (int j = size-1; j >= 0; --j) {
            distances[j].erase(distances[j].begin()+indexes[i]);
            bestPaths[j].erase(bestPaths[j].begin()+indexes[i]);
        }
    }
}

void Application::addClientPathToBaker(int source, Baker* baker, long int &currTime, Client* client)  {
    long int pathTime;
    int dest=client->getIndex();

    pathTime = addFloydPathToBaker( source, dest, baker);

    currTime = max(currTime + pathTime, client->getScheduledTime() - previousTolerance);
    client->setActualDeliveryTime(currTime);
    if(baker->getClientsAttended().empty())
        baker->setActualStartTime(currTime - pathTime);
    baker->addAttendedClient(client);
    if (capacityRestriction)
        baker->addBreadsDelivered(client->getBreadsQuantity());
    currTime += baker->getDeliveryTime();

}

long int Application::addFloydPathToBaker( int source, int dest, Baker* baker) {
    vector<int> path;
    vector<Edge<int> * > temp, convPath;
    long int pathTime;

    path = graph.getfloydWarshallPath(source,dest);
    convPath = convertNodeToEdges(path);
    pathTime = getPathTime(convPath);

    temp = baker->getPath();
    temp.insert(temp.end(),convPath.begin(),convPath.end());
    baker->setPath(temp);

    return pathTime;
}

void Application::multipleBakersDeliveryRun() {
    Csvfile file(DATA_FILE_PATH);

    if (algorithm) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleBakersDeliveryFloydBacktracking();
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Floyd MB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << bakerSet.size();
        file.endrow();
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    multipleBakersDeliveryDijkstraBacktracking();
    auto finish = std::chrono::high_resolution_clock::now();
    auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
    file << "Dijkstra MB" << graph.getVertexSet().size() << graph.getNumberEdges() << time
         << clientSet.size() << bakerSet.size();
    file.endrow();
}



void Application::multipleBakersDeliveryDijkstraBacktracking(){
    vector<Client *> clients(sortedClients());
    vector<Baker> bakers(sortedBakers()) , bestBakers;

    //remove clients that can't be reached by first baker
    auto it = clients.begin();
    while (it != clients.end()) {
        long int time = getPathTime(convertNodeToEdges(graph.getPath(bakery->getInfo(),(*it)->getVertex()->getInfo())));
        if(bakers.at(0).getStartTime() + time > (*it)->getScheduledTime() + afterTolerance) {
            it = clients.erase(it);
        } else {
            break;
        }
    }

    solveDijkstraBacktrackingMultipleBakers(clients,bakers, bestBakers);

    for(int i = 0 ; i < bakerSet.size(); i++){
        setDeliveryPathHoursWithDijkstra(bakerSet.at(i),bestBakers.at(i).getClientsAttended());
    }
}

void Application::solveDijkstraBacktrackingMultipleBakers(vector<Client * > sortedClients ,vector<Baker> currBakers,
                                                          vector<Baker> &bestBakers){

    if(sortedClients.empty()){
        if(bestBakers.empty()){
            bestBakers = vector<Baker>(currBakers);
            return;
        }
        long int nClientsBest = 0;
        long int nClientsCurr = 0;
        double bestPathTime = 0;
        double currPathTime = 0;
        for(int i = 0; i < currBakers.size(); i++){
            nClientsBest += bestBakers.at(i).getClientsAttended().size();
            nClientsCurr += currBakers.at(i).getClientsAttended().size();
            bestPathTime += bestBakers.at(i).getPathDuration();
            currPathTime += currBakers.at(i).getPathDuration();

        }
        bestPathTime = bestPathTime/(double) currBakers.size();
        currPathTime = currPathTime/(double) currBakers.size();
        if(nClientsCurr > nClientsBest) {
            bestBakers = vector<Baker>(currBakers);
        }else if(nClientsCurr == nClientsBest && currPathTime < bestPathTime){
            bestBakers = vector<Baker>(currBakers);
        }
        return;
    }


    //set up begin time
    for(int i = 0; i < currBakers.size(); i++){
        Baker * baker = &currBakers.at(i);
        if(baker->getClientsAttended().empty()) {
            baker->currTime = baker->getStartTime();
        }

    }

    Client * nextClient = sortedClients.at(0);
    sortedClients.erase(sortedClients.begin());
    solveDijkstraBacktrackingMultipleBakers(sortedClients,currBakers,bestBakers);

    //for each baker add the client
    for(int i = 0; i < currBakers.size(); i++) {
        vector<int> path;
        vector<Edge<int> * >convPath;
        long int tempTime;
        Baker *baker = &currBakers.at(i);
        Vertex<int> * currVert = getCurrBakerVertex(baker);



        graph.dijkstraTowPointsShortestPath(currVert->getInfo(), nextClient->getVertex()->getInfo());
        path = graph.getPath(currVert->getInfo(), nextClient->getVertex()->getInfo());
        convPath = convertNodeToEdges(path);

        if(baker->currTime + getPathTime(convPath) <= nextClient->getScheduledTime() + afterTolerance && (!capacityRestriction || (baker->getCapacity() >= nextClient->getBreadsQuantity() + baker->getBreadsDelivered()))){
            baker->addAttendedClient(nextClient);
            if(capacityRestriction){
                baker->addBreadsDelivered(nextClient->getBreadsQuantity());
            }

            long int currTime = baker->currTime;
            long int currPathTime = baker->getPathDuration();
            //verify time that it arrives to the next client
            tempTime = currTime;
            currTime -= currPathTime;
            currPathTime += getPathTime(convPath);
            currTime = max(currTime + currPathTime, nextClient->getScheduledTime() - previousTolerance);
            currPathTime += currTime - tempTime;
            currPathTime += baker->getDeliveryTime();
            currTime += baker->getDeliveryTime();

            baker->currTime = currTime;
            baker->setPathDuration(currPathTime);

            solveDijkstraBacktrackingMultipleBakers(sortedClients,currBakers,bestBakers);
            baker->removeLastAttendedClient();
        }
    }

}

Vertex<int> *Application::getCurrBakerVertex(Baker *baker) {
    if(baker->getClientsAttended().empty()){
        return bakery;
    }
    return baker->getClientsAttended().at(baker->getClientsAttended().size() - 1)->getVertex();
}

void Application::multipleBakersDeliveryFloydBacktracking() {
    vector<Client *> clientVec(sortedClients());
    for (int i = 0; i < bakerSet.size(); ++i) {
        multipleDeliveryFloydBacktracking(i, clientVec);
        vector<Client *> clients = bakerSet[i]->getClientsAttended();
        removeFromClientSet(clients, clientVec);
        if (clientVec.empty()) break;
    }
}

void Application::removeFromClientSet(vector<Client *> clients, vector<Client *> clientVec) {
    int i=0;
    while (!clients.empty()){
        //If not the same client
        if(!(clientVec[i]->getVertex()->getInfo() == clients[0]->getVertex()->getInfo())){ i++; continue; }

        clientVec.erase(clientVec.begin()+i);
        clients.erase(clients.begin());
        i=0;
    }
}

void Application::multipleBakersLimitedDeliveryRun() {
    Csvfile file(DATA_FILE_PATH);

    if (algorithm) {
        auto start = std::chrono::high_resolution_clock::now();
        multipleBakersDeliveryFloydBacktracking();
        auto finish = std::chrono::high_resolution_clock::now();
        auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
        file << "Floyd LD" << graph.getVertexSet().size() << graph.getNumberEdges() << time
             << clientSet.size() << bakerSet.size();
        file.endrow();
        return;
    }

    auto start = std::chrono::high_resolution_clock::now();
    multipleBakersDeliveryDijkstraBacktracking();
    auto finish = std::chrono::high_resolution_clock::now();
    auto time = chrono::duration_cast<chrono::milliseconds>(finish - start).count();
    file << "Dijkstra LD" << graph.getVertexSet().size() << graph.getNumberEdges() << time
         << clientSet.size() << bakerSet.size();
    file.endrow();
}


bool Application::solveLimitedFloydBacktracking(vector<int> &test, int currClient, vector<int> &res,
                                         vector<bool> canUse, long int currTime, int currQuantity,
                                         const vector<Client *> &clientVec,const int timeToDeliver) {
    vector<int> path;
    vector<Edge<int> * > convPath;
    long int pathTime;
    int available=0;

    Client* onGoing = clientVec[currClient];

    if (currQuantity==0) return true;

    for (int i = 0; i < clientVec.size(); ++i) {
        if (canUse[i]){
            Client * client = clientVec[i];

            if (currQuantity < client->getBreadsQuantity()){
                canUse[i]=false;
                continue;
            }

            //Find time to next client
            path = graph.getfloydWarshallPath(onGoing->getIndex(),client->getIndex());
            convPath = convertNodeToEdges(path);
            pathTime = getPathTime(convPath);

            //If baker can reach the client on time
            if(currTime + pathTime > client->getScheduledTime() + afterTolerance){
                canUse[i]= false;
            }
        }
        if(canUse[i]) {available++;}
    }

    //No more clients
    if(available==0) {
        if(test.size()>res.size()){
            res=vector<int>(test);
            return true;
        }
        if ( test.size()==res.size() && biggerQuantity(test,res,clientVec) ){
            res=vector<int>(test);
        }
        return true;
    }

    //Keep going
    for (int i = 0; i < clientVec.size(); ++i) {
        if(canUse[i]){
            Client * client = clientVec[i];

            //Insert new client on test
            test.push_back(client->getVertex()->getInfo());
            canUse[i] = false;

            //Get new currentTime
            path = graph.getfloydWarshallPath(onGoing->getIndex(),client->getIndex());
            convPath = convertNodeToEdges(path);
            pathTime = getPathTime(convPath);
            long int tmpTime = max(currTime + pathTime, client->getScheduledTime() - previousTolerance);

            solveLimitedFloydBacktracking(test, i, res, canUse, tmpTime+timeToDeliver,
                                          currQuantity-client->getBreadsQuantity(),clientVec, timeToDeliver);
            canUse[i]= true;
            test.pop_back();
        }
    }
    return true;
}

bool Application::biggerQuantity(const vector<int> &vec1, const vector<int> &vec2, const vector<Client *> &clients) {
    int j=0, k=0;
    int quantity1=0, quantity2=0;

    for (int i = 0; i < clients.size(); ++i) {
        Client* client=clients[i];

        //Founded all clients
        if (j==vec1.size()-1 && k==vec2.size()-1) break;

        //Client in vec1
        if(client->getVertex()->getInfo() == vec1[j]){
            quantity1+=client->getBreadsQuantity();
            j++;
        }

        //Client in vec2
        if(client->getVertex()->getInfo() == vec2[k]){
            quantity2+=client->getBreadsQuantity();
            k++;
        }
    }
    return quantity1>quantity2;
}

void Application::setCapacityRestriction(bool capacityRestriction) {
    Application::capacityRestriction = capacityRestriction;
}

void Application::setGraph(const Graph<int> &graph) {
    Application::graph = graph;
}

void Application::setInvertedGraph(const Graph<int> &invertedGraph) {
    Application::invertedGraph = invertedGraph;
}


