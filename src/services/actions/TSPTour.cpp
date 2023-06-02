#include "TSPTour.h"

TSPTour::TSPTour(Graph *&graph) : graph(&graph) {}

void TSPTour::execute() {


        vector<Edge> mst = (*graph)->findMinimumSpanningTree();

        for(auto edge : mst) {
            Node* orig = edge.getOrig();
            Node* dest = edge.getDest();

            orig->addMSTEdge(&edge);
            Edge reverseEdge = Edge(dest, orig, edge.getCapacity());
            dest->addMSTEdge(&reverseEdge);
        }

        Node* sourceNode = (*graph)->findNode(0);
        vector<Node*> HamiltonianPath = (*graph)->dfs(sourceNode);
        HamiltonianPath.push_back(sourceNode);

        double cost = 0;

        for(int i = 0; i < HamiltonianPath.size()-2; i++) {
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