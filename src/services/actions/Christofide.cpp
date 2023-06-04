//
// Created by francisco on 31-05-2023.
//

#include <future>
#include <csignal>
#include <fcntl.h>
#include "Christofide.h"
#include "../../view/DrawPaths.h"

Christofide::Christofide(Graph *&currentGraph) :graph(&currentGraph){};

void Christofide::execute() {

    pair<vector<Node*>,double>res=  (*graph)->christofidesTSP();
    pair<vector<int>,double> convert;
    convert.second= res.second;
    for(auto node: res.first){
        convert.first.push_back(node->getIndex());
    }
    DrawPaths().pageController(convert);



}