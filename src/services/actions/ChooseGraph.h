#ifndef DA_PROJECT1_CHOOSEGRAPH_H
#define DA_PROJECT1_CHOOSEGRAPH_H

#include "Action.h"
#include "../../view/Menu.h"

using namespace std;

class ChooseGraph : public Action
{
private:
    Graph **  currentGraph;
    Graph  newGraph;

public:
    ChooseGraph(Graph * &currentGraph, Graph &newGraph);

    void execute() override;
};

#endif // DA_PROJECT1_CHOOSEGRAPH_H
