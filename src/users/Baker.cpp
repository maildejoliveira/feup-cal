#include "Baker.h"

int Baker::idTracker=0;

Baker::Baker():id(idTracker) {idTracker++;}

Baker::Baker(long int startTime, long int  deliveryTime) : id(idTracker), startTime(startTime),
                                                      deliveryTime(deliveryTime) {idTracker++;}

Baker::Baker(long int  startTime, int capacity, long int  deliveryTime):id(idTracker), startTime(startTime),
                                                                  capacity(capacity),
                                                                  deliveryTime(deliveryTime) {idTracker++;}

long int  Baker::getStartTime() const {
    return startTime;
}

void Baker::setStartTime(long int  startTime) {
    Baker::startTime = startTime;
}

int Baker::getCapacity() const {
    return capacity;
}

void Baker::setCapacity(int capacity) {
    Baker::capacity = capacity;
}

long int  Baker::getDeliveryTime() const {
    return deliveryTime;
}

void Baker::setDeliveryTime(long int  deliveryTime) {
    Baker::deliveryTime = deliveryTime;
}

vector<Edge<int> *> &Baker::getPath() {
    return path;
}

void Baker::setPath(const vector<Edge<int> *> &path) {
    Baker::path = path;
}

void Baker::setActualStartTime(long actualStartTime) {
    Baker::actualStartTime = actualStartTime;
}

long Baker::getActualStartTime() const {
    return actualStartTime;
}

long Baker::getPathDuration() const {
    return pathDuration;
}

void Baker::setPathDuration(long pathDuration) {
    Baker::pathDuration = pathDuration;
}

const int Baker::getId() const {
    return id;
}

void Baker::addAttendedClient(Client *client) {
    clientsAttended.push_back(client);
}

vector<Client *> &Baker::getClientsAttended() {
    return clientsAttended;
}

void Baker::setClientsAttended(const vector<Client *> &clientsAttended) {
    Baker::clientsAttended = clientsAttended;
}

void Baker::removeLastAttendedClient() {
    clientsAttended.pop_back();
}

int Baker::getBreadsDelivered() const {
    return breadsDelivered;
}

void Baker::addBreadsDelivered(int breads) {
    breadsDelivered += breads;
}


