#ifndef CAL_PROJ_GRAPHLOADER_H
#define CAL_PROJ_GRAPHLOADER_H

#include <fstream>
#include "graphviewer.h"
#include "../graph/Graph.h"
#include <string>
#include <unistd.h>

class GraphLoader{
private:
    GraphViewer* gv;
    vector<sf::Color> colors{GraphViewer::MAGENTA,GraphViewer::YELLOW,GraphViewer::CYAN,GraphViewer::GREEN,GraphViewer::PINK,GraphViewer::BLUE,GraphViewer::ORANGE};
    int currColor = 0;
public:
    GraphLoader(Graph<int> &graph);
    void graphLoader();
    void setBakery(int idBakery);
    void join();
    void setPath(vector<Edge<int> *> &path);
    void setClients(vector<int> &clientsID);
    sf::Color getCurrColor();
    void nextColor();
};


#endif //CAL_PROJ_GRAPHLOADER_H
