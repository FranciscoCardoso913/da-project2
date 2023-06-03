#include "TriangularApproximation.h"
#include "../../view/DrawUtils.h"
#include "../../view/DrawPaths.h"

TriangularApproximation::TriangularApproximation(Graph *&graph) : graph(&graph) {}

void TriangularApproximation::execute() {


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
        pair<vector<int>,double> convert;

        double cost = 0;

        for(int i = 0; i < HamiltonianPath.size()-1; i++) {
            convert.first.push_back(HamiltonianPath[i]->getIndex());
            Edge* edge = (*graph)->findEdge(HamiltonianPath[i], HamiltonianPath[i+1]);
            if (edge == nullptr) {
                cost += (*graph)->calculateDistance(HamiltonianPath[i], HamiltonianPath[i+1]);
            }
            else cost += edge->getCapacity();
        }
        convert.first.push_back(HamiltonianPath.back()->getIndex());
        convert.second=cost;
        DrawPaths().pageController(convert);


}