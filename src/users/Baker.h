/*
 * Baker.h
 */
#ifndef BAKER_H_
#define BAKER_H_

#include "../graph/Edge.h"
#include <string>
#include "./Client.h"

template<class T> class Edge;
class Client;

class Baker {
    static int idTracker;
    const int id;           // id
    long int startTime;       // bakery departure utils
    long int actualStartTime;
    int capacity;           // amount of bread he can deliver
    long int deliveryTime;    // seconds spent in the processing of a delivery
    vector<Edge<int>*> path; // contains edges of the bakers path
    vector<Client *> clientsAttended; // contains the clients attended by the baker in order
    long int pathDuration = 0;
    int breadsDelivered = 0;

public:
    long int currTime = -1;

    Baker();

    void setClientsAttended(const vector<Client *> &clientsAttended);

    Baker(long int startTime, long int  deliveryTime);

    Baker(long int startTime, int capacity, long int  deliveryTime);

    long int getStartTime() const;

    void setStartTime(long int startTime);

    int getCapacity() const;

    void setCapacity(int capacity);

    long int getDeliveryTime() const;

    void setDeliveryTime(long int deliveryTime);

    vector<Edge<int> *> &getPath();

    void setPath(const vector<Edge<int> *> &path);

    void setActualStartTime(long actualStartTime);

    long getActualStartTime() const;

    long getPathDuration() const;

    void setPathDuration(long pathDuration);

    const int getId() const;

    void addAttendedClient(Client * client);

    vector<Client *> &getClientsAttended();

    void removeLastAttendedClient();

    int getBreadsDelivered() const;

    void addBreadsDelivered(int breads);
};


#endif /* Baker_ */
