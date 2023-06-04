#include "ChooseGraph.h"

ChooseGraph:: ChooseGraph(Graph * &currentGraph_, Graph &newGraph_)
{
    this->currentGraph = &currentGraph_;
    this->newGraph = newGraph_;
}

void ChooseGraph::execute()
{
    *currentGraph= &newGraph;
    cout<<"Graph has been changed!\n";
    wait();
}