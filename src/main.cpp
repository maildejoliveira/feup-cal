#include <iostream>
#include <graphviewer.h>
#include "graph/Graph.h"
#include "users/Client.h"
#include "./loaders/DataLoader.h"
#include "./utils/utils.h"
#include "./loaders/GraphLoader.h"
#include "./utils/Csvfile.h"

using namespace std;


int main(){
    //receber info

    DataLoader loader;
    Application application;


    cout << endl << "Welcome to Padaria of Sr. Silvio!!" << endl << endl
              << "\tWhat would you like to do?" << endl <<
              "\t\t1. One delivery with one van" << endl <<
              "\t\t2. Multiple deliveries with one van" << endl <<
              "\t\t3. Multiple deliveries with multiple vans" << endl <<
              "\t\t4. Multiple deliveries with multiple limited vans" << endl;

    int in;
    cin >> in;
    vector<Edge<int>*> path;

    switch (in) {
        case 1:
            application = loader.oneDeliveryInput();
            if(application.getClientsID().empty()) return 0;
            application.oneDeliveryRun();
            break;
        case 2:
            application = loader.multipleDeliveryInput();
            if(application.getClientsID().empty()) return 0;
            application.multipleDeliveryRun();
            break;
        case 3:
            application = loader.multipleBakersDeliveryInput();
            if(application.getClientsID().empty()) return 0;
            application.multipleBakersDeliveryRun();

            break;
        case 4:
            application = loader.multipleBakersLimitedDeliveryInput();
            if(application.getClientsID().empty()) return 0;
            application.multipleBakersLimitedDeliveryRun();
            break;
        default:
            cout << "Invalid input!";
    }


    //processar os dados

    //output

    GraphLoader gl(application.getGraph());
    gl.setBakery(application.getBakery()->getInfo());
    gl.setClients(application.getClientsID());
    cout << application.printPath();
    for(auto & i : application.getBakerSet()) {
        gl.setPath(i->getPath());
        gl.nextColor();
    }
    gl.join();
    return 0;
}
