//
// Created by francisco on 31-05-2023.
//

#include "Christofide.h"
#include "../../view/DrawPaths.h"

Christofide::Christofide(Graph *&currentGraph) :graph(&currentGraph){};

void Christofide::execute() {
    pair<vector<int>,double>res=  (*graph)->christofidesTSP();
    cout<<"Drawing?"<<endl;
    DrawPaths().pageController(res);
    /*vector<int> results;
    for(auto x:res.first){
        cout<<x<<'-';
    }
    cout<<endl<<res.second;

    wait();
    cout<<"Res"<<(*graph)->tspTriangularApproximation().second;*/

}