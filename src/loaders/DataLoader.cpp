#include <fstream>
#include <algorithm>
#include "DataLoader.h"
#include "../loaders/GraphLoader.h"

//Problems input

Application DataLoader::oneDeliveryInput() {

    //Input Graph Geral
    Application application;

    InputGraph(&application);

    //Bakery
    pair<Vertex<int>*, int> bakery=InputBakery(application.getGraph());
    application.setBakery(bakery.first);
    application.setBakeryIndex(bakery.second);

    //Baker
    application.setBakerSet(InputBaker(1));

    //Remove not convex nodes and edges
    application.preProcessGraphBFS();

    //Node can not supply any client
    if (!validGraph(application.getGraph())){
        return application;
    }

    //Tolerances
    InputTolerances(application);

    //One Client
    application.setClientSet(InputClients(application.getGraph(), 1, application));

    return application;
}

Application DataLoader::multipleDeliveryInput() {
    //Input Graph Geral
    Application application;

    InputGraph(&application);

    GraphLoader gl(application.getGraph());

    //Bakery
    pair<Vertex<int>*, int> bakery=InputBakery(application.getGraph());
    application.setBakery(bakery.first);
    application.setBakeryIndex(bakery.second);

    //Baker
    application.setBakerSet(InputBaker(1));

    //Remove not convex nodes and edges
    int i=0;
    do {
        cout << "What method would you like to use?" << endl <<
             "\t1. Dijkstar" << endl <<
             "\t2. Floyd Warshal" << endl;

        cin >> i;
    } while (i!=1 && i!=2);
    switch (i) {
        case 1:
            application.preProcessGraphBFS();
            application.setAlgorithm(false);
            break;
        case 2:
            application.preProcessGraphFloyd();
            application.setAlgorithm(true);
            break;
        default:
            break;
    }

    GraphLoader gl2(application.getGraph());
    gl2.setBakery(bakery.first->getInfo());

    //Node can not supply any client
    if (!validGraph(application.getGraph())){
        return application;
    }

    //Tolerances
    InputTolerances(application);

    //Multiple Clients
    int nClients=0;
    do{
        cout << "Insert number of clients:" << endl;
        cin >> nClients;
        cin.ignore(10000,'\n');
    } while (nClients<=0);
    application.setClientSet(InputClients(application.getGraph(), nClients, application));

    //application.setClientSet(readClientsFromFile(application.getGraph(),"../resources/clients"));

    return application;
}

Application DataLoader::multipleBakersDeliveryInput(){
    //Input Graph Geral
    Application application;

    InputGraph(&application);

    GraphLoader gl(application.getGraph());

    //Bakery
    pair<Vertex<int>*, int> bakery=InputBakery(application.getGraph());
    application.setBakery(bakery.first);
    application.setBakeryIndex(bakery.second);

    //Multiple bakers
    int nBakers=0;
    do{
        cout << "Insert number of Bakers:" << endl;
        cin >> nBakers;
        cin.ignore(10000,'\n');
    } while (nBakers <= 0);
    //Baker
    application.setBakerSet(InputBaker(nBakers));

    //Remove not convex nodes and edges
    int i=0;
    do {
        cout << "What method would you like to use?" << endl <<
             "\t1. Dijkstar" << endl <<
             "\t2. Floyd Warshal" << endl;

        cin >> i;
    } while (i!=1 && i!=2);
    switch (i) {
        case 1:
            application.preProcessGraphBFS();
            application.setAlgorithm(false);
            break;
        case 2:
            application.preProcessGraphFloyd();
            application.setAlgorithm(true);
            break;
        default:
            break;
    }

    GraphLoader gl2(application.getGraph());
    gl2.setBakery(bakery.first->getInfo());

    //Node can not supply any client
    if (!validGraph(application.getGraph())){
        return application;
    }

    //Tolerances
    InputTolerances(application);

    //Multiple Clients
    int nClients=0;
    do{
        cout << "Insert number of clients:" << endl;
        cin >> nClients;
        cin.ignore(10000,'\n');
    } while (nClients<=0);
    application.setClientSet(InputClients(application.getGraph(), nClients, application));

    //application.setClientSet(readClientsFromFile(application.getGraph(),"../resources/clients"));

    return application;
}

Application DataLoader::multipleBakersLimitedDeliveryInput() {
    //Input Graph Geral
    Application application;
    InputGraph( &application);

    application.setCapacityRestriction(true);

    GraphLoader gl(application.getGraph());

    //Bakery
    pair<Vertex<int>*, int> bakery=InputBakery(application.getGraph());
    application.setBakery(bakery.first);
    application.setBakeryIndex(bakery.second);

    //Multiple bakers with a capacity
    int nBakers=0;
    do{
        cout << "Insert number of Bakers:" << endl;
        cin >> nBakers;
        cin.ignore(10000,'\n');
    } while (nBakers <= 0);
    //Baker
    application.setBakerSet(InputBakerWithQuantity(nBakers));

    //Remove not convex nodes and edges
    int i=0;
    do {
        cout << "What method would you like to use?" << endl <<
             "\t1. Dijkstar" << endl <<
             "\t2. Floyd Warshal" << endl;

        cin >> i;
    } while (i!=1 && i!=2);
    switch (i) {
        case 1:
            application.preProcessGraphBFS();
            application.setAlgorithm(false);
            break;
        case 2:
            application.preProcessGraphFloyd();
            application.setAlgorithm(true);
            break;
        default:
            break;
    }

    GraphLoader gl2(application.getGraph());
    gl2.setBakery(bakery.first->getInfo());

    //Node can not supply any client
    if (!validGraph(application.getGraph())){
        return application;
    }

    //Tolerances
    InputTolerances(application);

    //Multiple Clients
    int nClients=0;
    do{
        cout << "Insert number of clients:" << endl;
        cin >> nClients;
        cin.ignore(10000,'\n');
    } while (nClients<=0);
    application.setClientSet(InputClients(application.getGraph(), nClients, application));

    //application.setClientSet(readClientsFromFile(application.getGraph(),"../resources/clients"));

    return application;
}

//Inputs

void DataLoader::InputGraph(Application * application) {

    Graph<int> graph, invertedGraph;
    bool doneReading = false;
    ifstream sStream;
    string path;
    int n;
    char trash;

    //Read graph nodes
    do {
        cout << "Insert graph nodes path:" << endl;
        cin >> path;
        cin.ignore(10000,'\n');

        sStream = ifstream(path);
        if (sStream.is_open()) {
            sStream >> n;
            for (int i = 0; i < n; ++i) {
                double x, y;
                int index;
                sStream >> trash >> index >> trash >> x >> trash >> y >> trash;

                //sStream >> index >> x >> y; // for testing maps

                graph.addVertex(index, x, y);
                invertedGraph.addVertex(index,x,y);
            }
            doneReading = true;
        }

        sStream.close();
    }while (!doneReading);

    doneReading = false;

    //Read graph edges
    do {
        cout << "Insert graph edges path:" << endl;
        cin >> path;
        cin.ignore(10000,'\n');

        sStream = ifstream(path);
        if (sStream.is_open()) {
            sStream >> n;
            for (int i = 0; i < n; ++i) {
                int source, dest, index;

                sStream >> trash >> /*index >> trash >>*/ source >> trash >> dest >> trash;

                //sStream >>/* index >>*/ source >> dest; // for testing maps

                graph.addEdge(source, dest, i);
                invertedGraph.addEdge(dest,source,i);

                /*if(i%3 == 0){
                    graph.addEdge(source, dest, i*2);
                    invertedGraph.addEdge(dest,source,i*2);

                    graph.addEdge(dest, source, i*2+1);
                    invertedGraph.addEdge(source,dest,i*2+1);
                }
                else{
                    graph.addEdge(source, dest, i*2);
                    invertedGraph.addEdge(dest,source,i*2);
                }*/

            }
            doneReading = true;
        }
        sStream.close();

    }while (!doneReading);

    application->setGraph(graph);
    application->setInvertedGraph(invertedGraph);
}

pair<Vertex<int>*, int> DataLoader::InputBakery(Graph<int> graph) {
    pair<Vertex<int>*, int> res(NULL, -1);

    do{
        cout << "Insert bakery vertex index:" << endl;
        int i;
        cin >> i;
        cin.ignore(10000,'\n');
        res=graph.findVertexAndIndex(i);
    } while (res.first==NULL);

    return res;
}

vector<Client * > DataLoader::InputClients(const Graph<int> &graph, int n, Application application) {
    vector<Client * > clients = vector<Client *>();
    long int startTime = application.getFirstBakerStart();
    long int tolerance = application.getAfterTolerance();
    int bakeryInfo = application.getBakery()->getInfo();

    for (int i = 0; i < n; ++i) {
        Client* client = new Client();
        pair<Vertex<int>*, int> v(NULL, -1);

        int index;
        do{
            cout << "Insert client vertex index:" << endl;

            cin >> index;
            cin.ignore(10000,'\n');
            cin.clear();

            if (find_if(clients.begin(), clients.end(), [index](Client* c){return c->getVertex()->getInfo()==index;})!=clients.end())
                continue;
            if (index!=bakeryInfo)
                v = graph.findVertexAndIndex(index);

        } while (v.first==NULL);

        client->setVertex(v.first);
        client->setIndex(v.second);

        long int seconds;

        do{
            string time;

            cout << "Insert scheduled time (HH:MM):" << endl;

            cin >> time;
            seconds = timeToSeconds(time);

            cin.ignore(10000,'\n');

        } while (seconds + tolerance <= startTime);

        client->setScheduledTime(seconds);


        int quantity;
        do{
            cout << "Insert amount of breads:" << endl;

            cin >> quantity;
            cin.ignore(10000,'\n');

        } while (quantity<=0);
        client->setBreadsQuantity(quantity);

        clients.push_back(client);
    }
    return clients;
}

vector<Baker * > DataLoader::InputBaker(const int n) {
    vector<Baker *> bakers = vector<Baker *>();

    for (int i = 0; i < n; ++i) {
        Baker* baker = new Baker();

        long int seconds;
        do{
            string time;

            cout << "Insert baker start time (HH:MM):" << endl;

            cin >> time;
            seconds = timeToSeconds(time);

            cin.ignore(10000,'\n');

        } while (seconds <= 0);

        baker->setStartTime(seconds);


        int time;
        do{
            cout << "Insert time spend in each delivery in minutes:" << endl;

            cin >> time;
            cin.ignore(10000,'\n');

        } while (time<=0);
        baker->setDeliveryTime(time * 60);

        bakers.push_back(baker);
    }
    return bakers;
}

void DataLoader::InputTolerances(Application &application) {
    int tolerance;
    do{
        cout << "Insert how many minutes before a scheduled time for a delivery it is accepted to happen:" << endl;

        cin >> tolerance;
        cin.ignore(10000,'\n');

    } while (tolerance<0);


    application.setPreviousTolerance(tolerance * 60);

    do{
        cout << "Insert how many minutes after a scheduled time for a delivery it is accepted to happen:" << endl;

        cin >> tolerance;
        cin.ignore(10000,'\n');

    } while (tolerance<0);
    application.setAfterTolerance(tolerance * 60);
}

bool DataLoader::validGraph(Graph<int> &graph) {
    if (graph.getNumVertex()==1){
        cout << endl << "Bakery can not supply any costumer service!" << endl;
        return false;
    }
    return true;
}

vector<Client * > DataLoader::readClientsFromFile(Graph<int> &graph,const std::string& file) {
    vector<Client * > clients = vector<Client *>();
    int n;
    ifstream ssFile(file);
    ssFile >> n;

    for (int i = 0; i < n; ++i) {
        Client* client = new Client();
        pair<Vertex<int>*, int> v(NULL, -1);

        int index;
        ssFile >> index;
        v = graph.findVertexAndIndex(index);
        client->setVertex(v.first);
        client->setIndex(v.second);

        long int seconds;
        string time;
        ssFile >> time;
        seconds = timeToSeconds(time);
        client->setScheduledTime(seconds);


        int quantity;
        ssFile >> quantity;
        client->setBreadsQuantity(quantity);


        clients.push_back(client);
    }
    return clients;
}

vector<Baker *> DataLoader::InputBakerWithQuantity(const int n) {
    vector<Baker *> bakers = vector<Baker *>();

    for (int i = 0; i < n; ++i) {
        Baker* baker = new Baker();

        long int seconds;
        do{
            string time;

            cout << "Insert baker start time (HH:MM):" << endl;

            cin >> time;
            seconds = timeToSeconds(time);

            cin.ignore(10000,'\n');

        } while (seconds <= 0);

        baker->setStartTime(seconds);


        int time;
        do{
            cout << "Insert time spend in each delivery in minutes:" << endl;

            cin >> time;
            cin.ignore(10000,'\n');

        } while (time<=0);
        baker->setDeliveryTime(time * 60);

        int quantity;
        do{
            cout << "Insert amount of breads it can deliver:" << endl;

            cin >> quantity;
            cin.ignore(10000,'\n');

        } while (quantity<=0);
        baker->setCapacity(quantity);

        bakers.push_back(baker);
    }
    return bakers;
}
