#include "NodesEdges.h"
#include "../view/DrawUtils.h"

Edge::Edge(Node *orig, Node *dest, double w) : orig(orig), dest(dest), capacity(w) {}

Node *Edge::getDest() const
{
    return this->dest;
}

int Edge::getCapacity() const
{
    return this->capacity;
}

Node *Edge::getOrig() const
{
    return this->orig;
}

Edge *Edge::getReverse() const
{
    return this->reverse;
}

bool Edge::isDisabled() const
{
    return this->disabled;
}

int Edge::getFlow() const
{
    return flow;
}


void Edge::setDisabled(bool _disabled)
{
    this->disabled = _disabled;
}

void Edge::setReverse(Edge *_reverse)
{
    this->reverse = _reverse;
}

void Edge::setFlow(int _flow)
{
    this->flow = _flow;
}


void Edge::setCapacity(int _capacity)
{
    this->capacity = _capacity;
}

