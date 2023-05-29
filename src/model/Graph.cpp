#include <stack>
#include "Graph.h"

Node * Graph::findNode(const int &index) const {
    for(auto node: nodes){
        if(node->getIndex()==index) return node;
    }
    return nullptr;
}
Line *Graph::findLine(const int &src, const int &dst) const
{
    for (Line *line : lines)
    {
        if (line->getOrig()->getIndex() == src && line->getDest()->getIndex() == dst)
        {
            return line;
        }
    }
    for (Line *line : lines)
    {
        if (line->getOrig()->getIndex() == dst && line->getDest()->getIndex() == src)
        {
            return line;
        }
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