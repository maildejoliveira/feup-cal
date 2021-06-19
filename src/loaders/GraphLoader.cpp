#include "GraphLoader.h"

using namespace std;
using Node = GraphViewer::Node;
using EdgeGV = GraphViewer::Edge;

void GraphLoader::graphLoader() {
    GraphViewer gv;
    //gv.setScale(1.0/4000.0);
    gv.setCenter(sf::Vector2f(300, 300));

    ifstream iNodesStream("../src/resources/Mapas-20210507/GridGraphs/GridGraphs/4x4/nodes.txt");
    GraphViewer::id_t N, idNode; int lat, lon; //BUT IT IS DOUBLE
    iNodesStream >> N;
    char trash;
    for(GraphViewer::id_t i = 0; i < N; ++i){
        iNodesStream >> trash >> idNode >> trash >> lat >> trash >> lon >> trash;
        Node &node = gv.addNode(idNode, sf::Vector2f(lat, lon));
        //node.setOutlineThickness(0.00002);
        //node.setSize(0.0001);
    }

    ifstream iEdgesStream("../src/resources/Mapas-20210507/GridGraphs/GridGraphs/4x4/edges.txt");
    GraphViewer::id_t E, u, v;
    iEdgesStream >> E;
    for(GraphViewer::id_t i = 0; i < E; ++i){
        iEdgesStream >> trash >> u  >> trash >> v >> trash;
        EdgeGV &edge = gv.addEdge(i, gv.getNode(u), gv.getNode(v), EdgeGV::EdgeType::DIRECTED);
        //edge.setThickness(0.0001);
        edge.setColor(GraphViewer::LIGHT_GRAY);
    }

    /*gv.setBackground(
            "../src/resources/map2/map.jpg",
            sf::Vector2f(-8.7817, -41.3095),
            sf::Vector2f(1.3297, 1.0)/7010.0f,
            0.8
    );*/

    // TODO
    //b.
    /*gv.setEnabledNodes(false); // Disable node drawing
    gv.setEnabledEdgesText(false); // Disable edge text drawing
    for (Node* node: gv.getNodes()) {
        node->setSize(0.0);
        node->setOutlineThickness(0.0);
    }

    //c.
    gv.setZipEdges(true);
    */

    gv.createWindow(1600, 900);
    gv.join();
}


GraphLoader::GraphLoader(Graph<int> &graph) {
    gv = new GraphViewer;
    int graphSize = graph.getVertexSet().size();
    //gv->setScale(1.0/40.0);
    gv->setCenter(sf::Vector2f(graph.getVertexSet().at(graphSize/2)->getX(), graph.getVertexSet().at(graphSize/2)->getY()));

    for(Vertex<int>* vertex: graph.getVertexSet()){
        gv->addNode(vertex->getInfo(), sf::Vector2f(vertex->getX(), vertex->getY())).setLabel(to_string(vertex->getInfo()));
    }

    for(Vertex<int>* vertex: graph.getVertexSet()){
        for(Edge<int> edge: vertex->getAdj()){
            gv->addEdge(edge.getId(),gv->getNode(vertex->getInfo()),gv->getNode(edge.getDest()->getInfo()), EdgeGV::EdgeType::DIRECTED);
        }
    }

    gv->createWindow(1600, 900);
}

void GraphLoader::setBakery(int idBakery) {
    gv->getNode(idBakery).setColor(GraphViewer::LIGHT_GRAY);
}


void GraphLoader::join() {
    gv->join();
}

void GraphLoader::setPath(vector<Edge<int>*> &path) {
    for(int i=0; i<path.size(); i++){
        gv->getEdge(path.at(i)->getId()).setColor(getCurrColor());
        usleep(path.at(i)->getWeight() * 100000);
    }

}

void GraphLoader::setClients(vector<int> &clientsID) {
    for(int id: clientsID){
        gv->getNode(id).setColor(GraphViewer::GREEN);
    }
}

sf::Color GraphLoader::getCurrColor() {
    return colors.at(currColor);
}

void GraphLoader::nextColor() {
    currColor = (currColor + 1) % colors.size();
}
