//
// Created by ze on 31-05-2023.
//

#include "Backtracking.h"

Backtracking::Backtracking(Graph *graph) : Action() {

    cout << "Backtracking" << endl;
    this->graph = graph;

}

void Backtracking::backtracking_tsp(int srcNode, int currNode, unsigned int graphSize, unsigned int count, unsigned int cost,
                                   unsigned int &minCost, vector<int> currPath , vector<int> &path) {

    if (count == graphSize){

        int finalCost = graph->findEdge(currPath[count-1], srcNode)->getCapacity();

        if ((cost + finalCost) < minCost) {
            minCost = cost+finalCost;

            for(int i = 0; i < count; i++) {
                path[i] = currPath[i];
            }
            path[count] = srcNode;

        }

        return;

    }


    for (int i = 0; i < graphSize; i++) {
        if(!graph->findNode(i)->isVisited()) {

            graph->findNode(i)->setVisited(true);

            currPath.push_back(i);

            backtracking_tsp(srcNode, i, graphSize, count+1, cost + graph->findEdge(currNode,i)->getCapacity(), minCost, currPath, path);

            graph->findNode(i)->setVisited(false);
        }
    }


}

void Backtracking::execute() {

    unsigned int minCost=std::numeric_limits<unsigned int>::max();
    int graphSize = graph->getNodes().size();

    string source;

    cout << "Insert the Source Station: ";
    cin.ignore(numeric_limits<streamsize>::max(), '\n');
    getline(cin, source);
    int srcNode = stoi(source);

    Node *sourceNode = graph->findNode(srcNode);

    if (sourceNode == nullptr) {
        cout << " Source doesn't exist";
        return;
    }

    for(auto node: graph->getNodes()) {
        node->setVisited(false);
    }

    sourceNode->setVisited(true);

    vector<int> path;
    vector<int> currPath;

    path.push_back(srcNode);


    backtracking_tsp(srcNode, 0, graphSize, 0, 0, minCost, currPath, path);




}