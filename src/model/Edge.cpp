#include "NodesEdges.h"

Edge::Edge(Node *orig, Node *dest, double w) : orig(orig), dest(dest), weight(w) {}

Node *Edge::getDest() const
{
    return this->dest;
}

double Edge::getWeight() const
{
    return this->weight;
}

Node *Edge::getOrig() const
{
    return this->orig;
}

Edge *Edge::getReverse() const
{
    return this->reverse;
}

void Edge::setReverse(Edge *_reverse)
{
    this->reverse = _reverse;
}

void Edge::setWeight(int weight)
{
    this->weight = weight;
}

