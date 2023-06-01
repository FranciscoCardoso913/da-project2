#include "NodesLine.h"
#include "../view/DrawUtils.h"

Node::Node(int index,double lon,double lat,string label) : index(index),lon(lon),lat(lat), label(label){};


/*
 * Auxiliary function to add an outgoing Line to a Station (this),
 * with a given destination Station (d) and Line capacity (w).
 */
Line *Node::addLine(Node *d, double w)
{
    auto newLine = new Line(this, d, w);
    adj.push_back(newLine);
    d->incoming.push_back(newLine);
    return newLine;
}

int Node::getIndex() const {
    return this->index;
}
/*
 * Auxiliary function to remove an outgoing Line (with a given destination (d))
 * from a Station (this).
 * Returns true if successful, and false if such Line does not exist.
 */
bool Node::removeLine(int index)
{
    bool removedLine = false;
    auto it = adj.begin();
    while (it != adj.end())
    {
        Line *Line = *it;
        Node *dest = Line->getDest();
        if (dest->getIndex() == index)
        {
            it = adj.erase(it);
            deleteLine(Line);
            removedLine = true; // allows for multiple Lines to connect the same pair of vertices (multigraph)
        }
        else
        {
            it++;
        }
    }
    return removedLine;
}

/*
 * Auxiliary function to remove an outgoing Line of a Station.
 */
void Node::removeOutgoingLines()
{
    auto it = adj.begin();
    while (it != adj.end())
    {
        Line *Line = *it;
        it = adj.erase(it);
        deleteLine(Line);
    }
}

bool Node::operator<(Node Station) const
{
    return this->queueIndex < Station.queueIndex;
}

std::vector<Line *> Node::getAdj() const
{
    return this->adj;
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

Line *Node::getPath() const
{
    return this->path;
}

std::vector<Line *> Node::getIncoming() const
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

void Node::setPath(Line *path)
{
    this->path = path;
}



void Node::deleteLine(Line *Line)
{
    Node *dest = Line->getDest();
    // Remove the corresponding Line from the incoming list
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
    delete Line;
}


void Node::setDisabled(bool _disabled)
{
    this->disabled = _disabled;
}

bool Node::isDisabled() const
{
    return this->disabled;
}


