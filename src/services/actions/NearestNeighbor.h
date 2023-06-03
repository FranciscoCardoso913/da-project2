//
// Created by francisco on 02-06-2023.
//

#ifndef DA_PROJECT2_NEARESTNEIGHBOR_H
#define DA_PROJECT2_NEARESTNEIGHBOR_H


#include "Action.h"

class NearestNeighbor: public Action{
public:
    /**
     * @brief Constructor to the NearestNeighbor object
     * @param graph graph being used
     */
    NearestNeighbor(Graph *&graph);
    /**
     * @brief Applies the Nearest Neighbor Algorithm to solve the TSP, and is the user gives permission upgrades the results
     * with a greedy algorithm and the LinKernighan algorithm
     * @complexity O( N²*2^n + E log E ) being N the number of nodes and E the number of edges
     */
    void execute() override;

private:

    Graph **graph;
};


#endif //DA_PROJECT2_NEARESTNEIGHBOR_H
