//
// Created by ze on 31-05-2023.
//

#ifndef DA_PROJECT2_BACKTRACKING_H
#define DA_PROJECT2_BACKTRACKING_H


#include "Action.h"

class Backtracking : public Action {

public:
    /**
     * @brief Constructor of the class Backtracking
     * @param graph
     */
    Backtracking(Graph &graph);


    void execute() override;


    void backtracking_tsp(Graph graph, int srcNode, int currNode, int n, int graphSize, int count, int cost, int &minCost);

};


#endif //DA_PROJECT2_BACKTRACKING_H
