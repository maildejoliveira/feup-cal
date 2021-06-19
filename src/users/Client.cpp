#include "Client.h"


int Client::idTracker=0;

Client::Client() : id(idTracker) {idTracker++; delivered = false;}

Client::Client(Vertex<int> *vertex, double scheduledTime) : id(0), vertex(vertex),
                                                            scheduledTime(scheduledTime) {
    idTracker++;
    delivered = false;
}

Client::Client(Vertex<int> *vertex, double scheduledTime, int breadsQuantity):id(0), vertex(vertex),
    scheduledTime(scheduledTime), breadsQuantity(breadsQuantity){
    idTracker++;
    delivered = false;}

void Client::setVertex(Vertex<int> *vertex) {
    Client::vertex = vertex;
}

void Client::setBreadsQuantity(int breadsQuantity) {
    Client::breadsQuantity = breadsQuantity;
}

void Client::setScheduledTime(long int scheduledTime) {
    Client::scheduledTime = scheduledTime;
}

Vertex<int> *Client::getVertex() const {
    return vertex;
}

long Client::getScheduledTime() const {
    return scheduledTime;
}

int Client::getBreadsQuantity() const {
    return breadsQuantity;
}

const int Client::getId() const {
    return id;
}

void Client::setActualDeliveryTime(long actualDeliveryTime) {
    Client::actualDeliveryTime = actualDeliveryTime;
}

long Client::getActualDeliveryTime() const {
    return actualDeliveryTime;
}

bool Client::isDelivered() const {
    return delivered;
}

void Client::setDelivered(bool delivered) {
    Client::delivered = delivered;
}

void Client::setIndex(int index) {
    Client::index = index;
}

int Client::getIndex() const {
    return index;
}

