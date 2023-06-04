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
    /**
     * @brief Action to choose a graph
     * @param currentGraph The current graph
     * @param newGraph The new graph to be chosen
     */
    ChooseGraph(Graph * &currentGraph, Graph &newGraph);

    /**
     * @brief Changes the current graph to the new graph
     * @brief Complexity: O(1)
     */
    void execute() override;
};

#endif // DA_PROJECT1_CHOOSEGRAPH_H
