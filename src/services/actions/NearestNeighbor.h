//
// Created by francisco on 02-06-2023.
//

#ifndef DA_PROJECT2_NEARESTNEIGHBOR_H
#define DA_PROJECT2_NEARESTNEIGHBOR_H


#include "Action.h"

class NearestNeighbor: public Action{
public:
    NearestNeighbor(Graph *&graph);

    void execute() override;

private:

    Graph **graph;
};


#endif //DA_PROJECT2_NEARESTNEIGHBOR_H
