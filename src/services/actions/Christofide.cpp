//
// Created by francisco on 31-05-2023.
//

#include "Christofide.h"

Christofide::Christofide(Graph *&currentGraph) :graph(&currentGraph){};

void Christofide::execute() {
    pair<vector<Node*>,double>res=  (*graph)->LinKernighan();
    (*graph)->reset();
    for(auto x:res.first){
        if(x->isVisited()) cout<<"Repetido"<<x->getIndex();
        x->setVisited(true);
    }
    for(auto x:res.first){
        if(!x->isVisited()) cout<<"Nop"<<x->getIndex();
    }
    for(auto x: res.first){
        cout<<x->getIndex()<<'-';
    }

    cout<<"Size:"<<res.first.size()<<endl;
    cout<<endl<<res.second;

    wait();
    //cout<<"Res"<<(*graph)->tspTriangularApproximation().second;

}