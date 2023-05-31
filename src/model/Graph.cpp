#include <stack>
#include "Graph.h"

Node *Graph::findNode(const int &index) const
{
    if (index >= nodes.size())
        return nullptr;
    return nodes[index];
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

vector<Node *> Graph::getNodes() const
{

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

int Graph::findParent(vector<int> &parent, int i)
{
    if (parent[i] == i)
        return i;
    return findParent(parent, parent[i]);
}

void Graph::mergeSets(vector<int> &parent, int x, int y)
{
    int xset = findParent(parent, x);
    int yset = findParent(parent, y);
    parent[xset] = yset;
}

vector<Line> Graph::findMinimumSpanningTree()
{
    vector<Line> result;
    vector<int> parent(this->getNodes().size());

    for (int i = 0; i < this->getNodes().size(); i++)
        parent[i] = i;

    int edgeCount = 0;
    int index = 0;

    while (edgeCount < this->getNodes().size() - 1)
    {
        Line *nextLine = this->getLineVector()[index++];
        Node *src = nextLine->getOrig();
        Node *dst = nextLine->getDest();

        int x = findParent(parent, src->getIndex());
        int y = findParent(parent, dst->getIndex());

        if (x != y)
        {
            result.emplace_back(src, dst, nextLine->getCapacity());
            mergeSets(parent, x, y);
            edgeCount++;
        }
    }
    return result;
}