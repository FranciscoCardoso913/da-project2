#ifndef DA_PROJECT2_TRIANGULARAPPROXIMATION_H
#define DA_PROJECT2_TRIANGULARAPPROXIMATION_H


#include "Action.h"

class TriangularApproximation : public Action {

public:
    /**
     * @brief Constructor of the Action that executes the Triangular Approximation algorithm
     * @param graph current graph
     */
    TriangularApproximation(Graph * &graph);

    /**
     * @brief Executes the Triangular Approximation algorithm
     * @brief Complexity: O(N+E) - N is the number of Nodes and E is the number of edges
     */
    void execute() override;

private:
        Graph **graph;


};


#endif //DA_PROJECT2_TRIANGULARAPPROXIMATION_H
