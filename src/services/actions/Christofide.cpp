//
// Created by francisco on 31-05-2023.
//

#include <future>
#include <csignal>
#include <fcntl.h>
#include "Christofide.h"

Christofide::Christofide(Graph *&currentGraph) :graph(&currentGraph){};

void Christofide::execute() {
    bool run=true;
    double solution=0;
    int original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    auto func = [&]( bool * run, double * solution)
    { return (*graph)->LinKernighan(run , solution);};
    future<pair<vector<Node*>,double>> f = async(func, &run,&solution);
    time_t start = ::time(NULL);
    time_t curr = 0;
    int currSolution = solution;
    while (run)
    {

        if ((time(NULL) - curr) > 0 or currSolution   >= solution+1)
        {
            char c='\0';
            if (read(STDIN_FILENO, &c, 1) == 1) {
                if(c!='\0') run= false;
            }
            currSolution = solution;
            curr = time(NULL);
            system("clear");
            cout << "Improving Solution" << endl;
            cout <<"Solution:"<< solution ;
            cout << "          " << curr - start << " s" << endl;
            cout<<"Press enter to finish improvements"<<endl;
        }
    }
    fcntl(STDIN_FILENO, F_SETFL, original_flags);
    pair<vector<Node*>,double>res=  f.get();

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