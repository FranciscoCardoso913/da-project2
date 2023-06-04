#include "NodesEdges.h"
#include "../view/DrawUtils.h"

Node::Node(int index,double lon,double lat) : index(index),lon(lon),lat(lat){};

bool Node::operator<(Node node) const
{
    return this->dist < node.dist;
}

std::vector<Edge *> Node::getAdj() const
{
    return this->adj;
}

std::vector<Edge *> Node::getMST() const
{
    return this->mst;
}

int Node::getIndex() const {
    return this->index;
}

bool Node::isVisited() const
{
    return this->visited;
}

bool Node::isProcessing() const
{
    return this->processing;
}

double Node::getDist() const
{
    return this->dist;
}

Edge* Node::getPath() const
{
    return this->path;
}

void Node::setVisited(bool visited)
{
    this->visited = visited;
}

void Node::setProcessing(bool processing)
{
    this->processing = processing;
}

void Node::setDist(double dist)
{
    this->dist = dist;
}

void Node::setPath(Edge *path)
{
    this->path = path;
}

Edge *Node::addEdge(Node *dest, double w)
{
    auto newEdge = new Edge(this, dest, w);
    adj.push_back(newEdge);
    dest->incoming.push_back(newEdge);
    return newEdge;
}

void Node::addMSTEdge(Edge *mstEdge) {
    mst.push_back(mstEdge);
}


