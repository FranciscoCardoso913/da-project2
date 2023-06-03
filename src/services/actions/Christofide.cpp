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
    pair<vector<Node*>,double> initial_solution= (*graph)->tspTriangularApproximation();
    solution=initial_solution.second;
    int original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    auto func = [&]( bool * run, double * solution,pair<vector<Node *>, double> *initialTour )
    {
        if(!*run) return ;
        (*graph)->greddyImprovement(run,solution,*initialTour);
        if(!*run) return ;
        (*graph)->LinKernighan(run , solution,*initialTour);
        *run= false;
    };
    future<void> f = async(func, &run,&solution,&initial_solution);
    time_t start = ::time(NULL);
    time_t curr = 0;

    double currSolution = 0;
    while (run)
    {
        if ((time(NULL) - curr) > 0 or currSolution   != solution)
        {


            char c='\0';
            if (read(STDIN_FILENO, &c, 1) == 1) {
                if(c!='\0') run= false;
            }

            currSolution = initial_solution.second;
            curr = time(NULL);
            system("clear");
            cout << "Improving Solution" << endl;
            cout <<"Solution:"<< solution ;
            cout << "          " << curr - start << " s" << endl;
            cout<<"Press enter to finish improvements"<<endl;
        }
    }
    fcntl(STDIN_FILENO, F_SETFL, original_flags);
    f.get();

    (*graph)->reset();
    for(auto x:initial_solution.first){
        if(x->isVisited()) cout<<"Repetido"<<x->getIndex();
        x->setVisited(true);
    }
    for(auto x:initial_solution.first){
        if(!x->isVisited()) cout<<"Nop"<<x->getIndex();
    }
    for(auto x: initial_solution.first){
        cout<<x->getIndex()<<'-';
    }

    cout<<"Size:"<<initial_solution.first.size()<<endl;
    cout<<endl<<initial_solution.second;

    wait();

    //cout<<"Res"<<(*graph)->tspTriangularApproximation().second;

}