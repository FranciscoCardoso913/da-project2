//
// Created by francisco on 31-05-2023.
//

#include "Christofide.h"

Christofide::Christofide(Graph *&currentGraph) :graph(currentGraph){};

void Christofide::execute() {
    pair<vector<int>,int>res=  graph->christofidesSTP();
    for(int x:res.first){
        cout<<x<<'-';
    }
    cout<<'\n';
    cout<<"Weight : "<<res.second<<endl;
    wait();
}