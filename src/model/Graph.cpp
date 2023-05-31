#include <stack>
#include "Graph.h"

Node * Graph::findNode(const int &index) const {
    if(index>=nodes.size()) return nullptr;
    return nodes[index];
}
Line *Graph::findLine( Node *src,  Node *dst) const
{
    for(auto line: src->getAdj()){
        if(line->getDest()==dst) return line;
    }
    return nullptr;
}

bool Graph::addNode(Node *node)
{
    nodes.push_back(node);
    return true;
}

bool Graph::addLine(Node *src, Node *dest, int w)
{
    if (src == nullptr || dest == nullptr)
        return false;
    src->addLine(dest, w);
    return true;
}


vector<Line *> Graph::getLineVector() const
{
    return lines;
}

bool Graph::addBidirectionalLine(Node *src, Node *dst, double w)
{
    if (src == nullptr || dst == nullptr)
        return false;
    auto e1 = src->addLine(dst, w);
    auto e2 = dst->addLine(src, w);
    lines.push_back(e1);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}

vector<Node *> Graph::getNodes() const {

    return nodes;
}
void Graph::reset()
{
    for (Node *node : nodes)
    {
        node->setVisited(false);
        node->setProcessing(false);
        for (Line *line : node->getAdj())
        {
            line->setFlow(0);
        }
    }
}
vector<int> Graph::oddDegreeVertices(vector<Line> lines) const {
    std::vector<int> oddDegreeVertices;
    std::vector<int> degreeCount(this->nodes.size(), 0);
    for (const auto& edge : lines) {
        degreeCount[edge.getOrig()->getIndex()]++;
        degreeCount[edge.getDest()->getIndex()]++;
    }
    for (int i = 0; i < this->nodes.size(); i++) {
        if (degreeCount[i] % 2 == 1) {
            oddDegreeVertices.push_back(i);
        }
    }
    return oddDegreeVertices;
}
vector<Line > Graph:: minimumPerfectMatching (vector<int> oddNodes) {
    std::vector<Line> perfectMatching;
    reset();
    for (int i = 0; i < oddNodes.size() - 1; i += 2) {
        Node* src = findNode(oddNodes[i]) ;
        Node* dest = findNode(oddNodes[i+1]) ;
        int weight = findLine(src,dest)->getCapacity();
        perfectMatching.push_back(Line(src, dest,weight));
        src->setVisited(true);
        dest->setVisited(true);
    }

    int unmatchedVertex = -1;
    for (int i = 0; i < nodes.size(); i++) {
        if (!findNode(i)) {
            unmatchedVertex = i;
            break;
        }
    }

    if (unmatchedVertex != -1) {
        int minWeight = std::numeric_limits<int>::max();
        int closestVertex = -1;

        for (int i = 0; i < oddNodes.size(); i++) {
            if (oddNodes[i] != unmatchedVertex && findLine(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() < minWeight) {
                minWeight = findLine(findNode(unmatchedVertex), findNode(oddNodes[i]))->getCapacity() ;
                closestVertex = oddNodes[i];
            }
        }

        perfectMatching.push_back(Line(findNode(unmatchedVertex), findNode(closestVertex), minWeight));
    }

    return perfectMatching;
}
vector<int> Graph:: findEulerianCircuit(vector<Line> edges ) {
    std::vector<int> circuit;
    std::vector<std::vector<int>> adjList(nodes.size());

    for (const auto& edge : edges) {
        adjList[edge.getOrig()->getIndex()].push_back(edge.getDest()->getIndex());
        adjList[edge.getDest()->getIndex()].push_back(edge.getOrig()->getIndex());
    }
    int currVertex = 0;
    circuit.push_back(currVertex);

    while (!adjList[currVertex].empty()) {
        int nextVertex = adjList[currVertex].back();
        adjList[currVertex].pop_back();

        auto it = std::find(adjList[nextVertex].begin(), adjList[nextVertex].end(), currVertex);
        adjList[nextVertex].erase(it);

        circuit.push_back(nextVertex);
        currVertex = nextVertex;
    }

    return circuit;
}
vector<int> Graph:: tspTours(vector<int> &eulerianCircuit){
    std::vector<int> tspTour;
    reset();
    for (int node : eulerianCircuit) {
        if (!findNode(node)->isVisited()) {
            tspTour.push_back(node);
            findNode(node)->setVisited(true);
        }
    }

    return tspTour;
}


void deleteMatrix(int **m, int n)
{
    if (m != nullptr)
    {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete[] m[i];
        delete[] m;
    }
}

void deleteMatrix(double **m, int n)
{
    if (m != nullptr)
    {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete[] m[i];
        delete[] m;
    }
}

Graph::~Graph()
{
    deleteMatrix(distMatrix, nodes.size());
    deleteMatrix(pathMatrix, nodes.size());

}