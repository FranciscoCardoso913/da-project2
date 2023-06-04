//
// Created by francisco on 31-05-2023.
//

#ifndef DA_PROJECT2_CHRISTOFIDE_H
#define DA_PROJECT2_CHRISTOFIDE_H


#include "Action.h"
#include "../../view/Menu.h"


using namespace std;

class Christofide : public Action
{
private:
    Graph **  graph;


public:
    /**
     * @brief Constructor of the Christofides object
     * @param currentGraph pointer that points to the graph being used
     */
    Christofide(Graph * &currentGraph);
    /**
     * @brief Applies the Christofides Algorithm to solve the TSP, and is the user gives permission upgrades the results
     * with a greedy algorithm and the LinKernighan algorithm
     * @complexity O( N²*2^n + E (log E + log N) ) being N the number of nodes and E the number of edges
     */
    void execute() override;
};


#endif //DA_PROJECT2_CHRISTOFIDE_H
