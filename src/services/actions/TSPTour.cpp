#include "TSPTour.h"

TSPTour::TSPTour(Graph *&graph) : graph(&graph) {}

void TSPTour::execute() {


        Node* sourceNode = (*graph)->findNode(0);
        vector<Edge*> mst = (*graph)->findMinimumSpanningTree(sourceNode);

        for(auto edge : mst) {
            Node* orig = edge->getOrig();
            Node* dest = edge->getDest();


            orig->addMSTEdge(edge);
            dest->addMSTEdge(edge->getReverse());
        }



        vector<Node*> HamiltonianPath;

        (*graph)->reset();

        (*graph)->dfs(sourceNode, HamiltonianPath);

        HamiltonianPath.push_back(sourceNode);

        double cost = 0;

        for(int i = 0; i < HamiltonianPath.size()-1; i++) {
            Edge* edge = (*graph)->findEdge(HamiltonianPath[i], HamiltonianPath[i+1]);
            cost += edge->getCapacity();
        }

        cout << "Cost: " << cost << endl;
        cout << "Path";
        for (int i = 0; i < HamiltonianPath.size(); i++) {
            cout << " -> " << HamiltonianPath[i]->getIndex();
        }
        cout << endl;


}