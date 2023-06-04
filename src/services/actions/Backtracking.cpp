
#include "Backtracking.h"

#include "../../view/DrawPaths.h"

#include "../../view/DrawUtils.h"


Backtracking::Backtracking(Graph *&graph):  graph(&graph) {};

void Backtracking::backtracking_tsp(int srcNode, int currNode, unsigned int graphSize, unsigned int count, double cost,
                                   double &minCost, vector<int> currPath , vector<int> &path) {


    if (count == graphSize && (*graph)->findEdge((*graph)->findNode(currPath[count-1]), (*graph)->findNode(srcNode))!=nullptr) {

        Edge *finalEdge = (*graph)->findEdge((*graph)->findNode(currPath[count-1]), (*graph)->findNode(srcNode));

        double finalCost = finalEdge->getWeight() + cost;

        if (finalCost < minCost) {
            minCost = finalCost;

            path.clear();
            for(int i = 0; i < currPath.size(); i++) {
                path.push_back(currPath[i]);
            }
            path.push_back(srcNode);

        }

        return;

    }


    for (int i = 0; i < graphSize; i++) {

        Node *node = (*graph)->findNode(i);
        Node *currentNode = (*graph)->findNode(currNode);
        Edge *edge = (*graph)->findEdge(currentNode,node);

        if(!node->isVisited() && edge!=NULL) {

            node->setVisited(true);

            currPath.push_back(i);

            backtracking_tsp(srcNode, i, graphSize, count+1, cost + edge->getWeight(), minCost, currPath, path);

            node->setVisited(false);

            currPath.pop_back();
        }
    }


}

void Backtracking::execute() {

    double minCost = std::numeric_limits<double>::max();
    int graphSize = (*graph)->getNodes().size();

    string source;
    while (true) {
        cout << "Insert the Source node: ";
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        getline(cin, source);
        if (source.empty()) {
            cout << "Invalid Input! \n";
        } else {
            break;
        }
    }
    int srcNode = stoi(source);

    Node *sourceNode = (*graph)->findNode(srcNode);

    if (sourceNode == nullptr) {
        cout << " Source doesn't exist! \n";
        return;
    }

    for(auto node: (*graph)->getNodes()) {
        node->setVisited(false);
    }

    sourceNode->setVisited(true);

    vector<int> path;
    vector<int> currPath;

    path.push_back(srcNode);
    currPath.push_back(srcNode);

    backtracking_tsp(srcNode, srcNode, graphSize, 1, 0, minCost, currPath, path);


    DrawPaths().pageController(make_pair(path,minCost));



}