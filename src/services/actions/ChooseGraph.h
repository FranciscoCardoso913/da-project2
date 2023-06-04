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
     * @brief Constructor of the ChooseGraph object
     * @param currentGraph Pointer pointing to the current Graph
     * @param newGraph the new Graph the pointer will point to
     */
    ChooseGraph(Graph * &currentGraph, Graph &newGraph);

    /**
     * @brief Changes the current graph to the new graph
     * @brief Complexity: O(1)
     */
    void execute() override;
};

#endif // DA_PROJECT1_CHOOSEGRAPH_H
