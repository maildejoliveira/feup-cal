#ifndef CAL_PROJ_DATALOADER_H
#define CAL_PROJ_DATALOADER_H


#include "../graph/Graph.h"
#include "../Application.h"
#include "../utils/utils.h"
#include <string>

class DataLoader {

public:

    Application oneDeliveryInput();

    void InputGraph(Application * application);

    pair<Vertex<int>*, int> InputBakery(Graph<int> graph);

    vector<Client *> InputClients(const Graph<int> &graph, int n, Application application);

    void InputTolerances(Application &application);

    vector<Baker *> InputBaker( int n);

    bool validGraph(Graph<int> &graph);

    Application multipleDeliveryInput();

    vector<Client * > readClientsFromFile(Graph<int> &graph,const std::string& file);

    Application multipleBakersDeliveryInput();

    Application multipleBakersLimitedDeliveryInput();

    vector<Baker *> InputBakerWithQuantity(const int n);
};


#endif //CAL_PROJ_DATALOADER_H
