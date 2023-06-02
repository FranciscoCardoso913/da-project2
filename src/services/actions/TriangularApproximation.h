#ifndef DA_PROJECT2_TRIANGULARAPPROXIMATION_H
#define DA_PROJECT2_TRIANGULARAPPROXIMATION_H


#include "Action.h"

class TriangularApproximation : public Action {

public:
    TriangularApproximation(Graph * &graph);

    void execute() override;

private:
        Graph **graph;


};


#endif //DA_PROJECT2_TRIANGULARAPPROXIMATION_H
