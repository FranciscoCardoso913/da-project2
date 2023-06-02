//
// Created by francisco on 02-06-2023.
//

#include "NearestNeighbor.h"
#include "../../view/DrawPaths.h"

NearestNeighbor::NearestNeighbor(Graph *&graph) :graph(&graph){}

void NearestNeighbor::execute() {
    DrawPaths().pageController((*graph)->nearestNeighborTSP());
}