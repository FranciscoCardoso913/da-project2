#include "NodesEdges.h"
#include "../view/DrawUtils.h"

Node::Node(int index,double lon,double lat,string label) : index(index),lon(lon),lat(lat), label(label){};


/*
 * Auxiliary function to add an outgoing Edge to a Station (this),
 * with a given destination Station (d) and Edge capacity (w).
 */
Edge *Node::addEdge(Node *d, double w)
{
    auto newEdge = new Edge(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

bool Node::addMSTEdge(Edge *mstEdge) {
    mst.push_back(mstEdge);
    return 1;
}

int Node::getIndex() const {
    return this->index;
}
/*
 * Auxiliary function to remove an outgoing Edge (with a given destination (d))
 * from a Station (this).
 * Returns true if successful, and false if such Edge does not exist.
 */
bool Node::removeEdge(int index)
{
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end())
    {
        Edge *Edge = *it;
        Node *dest = Edge->getDest();
        if (dest->getIndex() == index)
        {
            it = adj.erase(it);
            deleteEdge(Edge);
            removedEdge = true; // allows for multiple Edges to connect the same pair of vertices (multigraph)
        }
        else
        {
            it++;
        }
    }
    return removedEdge;
}

/*
 * Auxiliary function to remove an outgoing Edge of a Station.
 */
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

std::vector<Edge *> Node::getIncoming() const
{
    return this->incoming;
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


void Node::setDisabled(bool _disabled)
{
    this->disabled = _disabled;
}

bool Node::isDisabled() const
{
    return this->disabled;
}


