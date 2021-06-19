/*
 * Client.h
 */
#ifndef CLIENT_H_
#define CLIENT_H_

#include "../graph/Vertex.h"

template<class T> class Vertex;

class Client {
    static int idTracker;
private:
    const int id;
    Vertex<int> * vertex;   // destination vertex
    int index;
    long int scheduledTime;   // time of delivery in seconds
    int breadsQuantity;
    long int actualDeliveryTime; //seconds of when the delivery happened
    bool delivered;
public:
    void setBreadsQuantity(int breadsQuantity);
    // amount of bread to be delivered
    Client();

    Client(Vertex<int> *vertex, double scheduledTime);

    Client(Vertex<int> *vertex, double scheduledTime, int breadsQuantity);

    void setVertex(Vertex<int> *vertex);

    int getIndex() const;

    void setIndex(int index);

    void setScheduledTime(long int scheduledTime);

    Vertex<int> *getVertex() const;

    long getScheduledTime() const;

    int getBreadsQuantity() const;

    const int getId() const;

    void setActualDeliveryTime(long actualDeliveryTime);

    long getActualDeliveryTime() const;

    bool isDelivered() const;

    void setDelivered(bool delivered);
};


#endif /* Client_ */
