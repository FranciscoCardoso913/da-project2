//
// Created by ze on 18-03-2023.
//


#include "ChooseGraph.h"

ChooseGraph:: ChooseGraph(Graph * &currentGraph_, Graph &newGraph_)
{
    this->currentGraph = &currentGraph_;
    this->newGraph = newGraph_;
}

void ChooseGraph::execute()
{
    *currentGraph= &newGraph;
}