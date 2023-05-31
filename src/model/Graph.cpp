#include <stack>
#include "Graph.h"

Node * Graph::findNode(const int &index) const {
    if(index>=nodes.size()) return nullptr;
    return nodes[index];
}
Edge *Graph::findEdge(const int &src, const int &dst) const
{
    for (Edge *edge : edges)
    {
        if (edge->getOrig()->getIndex() == src && edge->getDest()->getIndex() == dst)
        {
            return edge;
        }
    }
    for (Edge *edge : edges)
    {
        if (edge->getOrig()->getIndex() == dst && edge->getDest()->getIndex() == src)
        {
            return edge;
        }
    }
    return nullptr;
}

bool Graph::addNode(Node *node)
{
    nodes.push_back(node);
    return true;
}

bool Graph::addEdge(Node *src, Node *dest, int w)
{
    if (src == nullptr || dest == nullptr)
        return false;
    src->addEdge(dest, w);
    return true;
}


vector<Edge *> Graph::getEdgeVector() const
{
    return edges;
}

bool Graph::addBidirectionalEdges(Node *src, Node *dst, double w)
{
    if (src == nullptr || dst == nullptr)
        return false;
    auto e1 = src->addEdge(dst, w);
    auto e2 = dst->addEdge(src, w);
    edges.push_back(e1);
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
        for (Edge *edge : node->getAdj())
        {
            edge->setFlow(0);
        }
    }
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