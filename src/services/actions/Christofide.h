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
    Christofide(Graph * &currentGraph);

    void execute() override;
};


#endif //DA_PROJECT2_CHRISTOFIDE_H
