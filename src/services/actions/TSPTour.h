#ifndef DA_PROJECT2_TSPTOUR_H
#define DA_PROJECT2_TSPTOUR_H


#include "Action.h"

class TSPTour : public Action {

public:
    TSPTour(Graph * &graph);

    void execute() override;

private:
        Graph **graph;


};


#endif //DA_PROJECT2_TSPTOUR_H
