
#ifndef DA_PROJECT2_BACKTRACKING_H
#define DA_PROJECT2_BACKTRACKING_H


#include "Action.h"

class Backtracking : public Action {

public:
    /**
     * @brief Constructor of the class Backtracking
     * @param graph
     */
    Backtracking(Graph *&graph);


    void execute() override;


    void backtracking_tsp(int srcNode, int currNode, unsigned int graphSize, unsigned int count, double cost, double &minCost, vector<int> currPath, vector<int> &path);

private:

    Graph **graph;

};


#endif //DA_PROJECT2_BACKTRACKING_H
