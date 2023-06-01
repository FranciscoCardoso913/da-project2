//
// Created by francisco on 31-05-2023.
//

#include "Christofide.h"

Christofide::Christofide(Graph *&currentGraph) :graph(&currentGraph){};

void Christofide::execute() {
    /*pair<vector<int>,double>res=  (*graph)->christofidesTSP();
    for(auto x:res.first){
        cout<<x<<'-';
    }
    cout<<endl<<res.second;

    wait();*/
    cout<<"Res"<<(*graph)->tspTriangularApproximation().second;

}