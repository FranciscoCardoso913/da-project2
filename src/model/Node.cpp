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

double Node::getLon()const{return this->lon;}

double Node::getLat() const{return this->lat;}

int Node:: getTSPIndex() const{return this->tspIndex;}

void Node::setTSPIndex(int TSPIndex){
    this->tspIndex=TSPIndex;
}

void Node::removeOutgoingEdges()
{
    auto it = adj.begin();
    while (it != adj.end())
    {
        Edge *Edge = *it;
        it = adj.erase(it);
        deleteEdge(Edge);
    }
}
void Node::deleteEdge(Edge *Edge)
{
    Node *dest = Edge->getDest();
    // Remove the corresponding Edge from the incoming list
    auto it = dest->incoming.begin();
    while (it != dest->incoming.end())
    {
        if ((*it)->getOrig()->getIndex() == this->index)
        {
            it = dest->incoming.erase(it);
        }
        else
        {
            it++;
        }
    }
    delete Edge;
}




