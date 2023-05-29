#include "NodesLine.h"
#include "../view/DrawUtils.h"

Line::Line(Node *orig, Node *dest, double w) : orig(orig), dest(dest), capacity(w) {}

Node *Line::getDest() const
{
    return this->dest;
}

int Line::getCapacity() const
{
    return this->capacity;
}

Node *Line::getOrig() const
{
    return this->orig;
}

Line *Line::getReverse() const
{
    return this->reverse;
}

bool Line::isDisabled() const
{
    return this->disabled;
}

int Line::getFlow() const
{
    return flow;
}


void Line::setDisabled(bool _disabled)
{
    this->disabled = _disabled;
}

void Line::setReverse(Line *_reverse)
{
    this->reverse = _reverse;
}

void Line::setFlow(int _flow)
{
    this->flow = _flow;
}


void Line::setCapacity(int _capacity)
{
    this->capacity = _capacity;
}

