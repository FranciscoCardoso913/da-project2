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

    /**
     * @brief Method that executes the backtracking algorithm
     * @brief Complexity: O(n!) where n is the number of nodes
     * @param srcNode The source node
     * @param currNode The current node
     * @param graphSize the size of the graph
     * @param count the number of nodes in path
     * @param cost the cost of current path
     * @param minCost the minimum cost reached
     * @param currPath the current checked path
     * @param path the minimum cost path
     */
    void backtracking_tsp(int srcNode, int currNode, unsigned int graphSize, unsigned int count, double cost, double &minCost, vector<int> currPath, vector<int> &path);

    /**
     * @brief Method that executes the Backtracking Menu. It asks for the source, and after resetting the graph, it executes the backtracking algorithm.
     * @brief Complexity: O(n!) where n is the number of nodes
     */
    void execute() override;

private:

    Graph **graph;

};


#endif //DA_PROJECT2_BACKTRACKING_H
