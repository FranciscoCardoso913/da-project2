//
// Created by francisco on 02-06-2023.
//

#include "NearestNeighbor.h"
#include "../../view/DrawPaths.h"
#include <future>
#include <csignal>
#include <fcntl.h>
#include "Christofide.h"
#include "../../view/DrawPaths.h"

NearestNeighbor::NearestNeighbor(Graph *&graph) :graph(&graph){}

void NearestNeighbor::execute() {
    bool run=false;

    double solution=0;
    ::system("clear");
    cout<<"Calculating solution:\n";
    pair<vector<Node*>,double> initial_solution= (*graph)->nearestNeighborTSP();
    solution=initial_solution.second;
    cout<<"Solution obtained:"<<solution<<endl;
    cout<<"Upgrade Solution? [y/n]";
    string res;
    cin>>res;
    if(res=="y" || res=="Y") run= true;
    int original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    auto func = [&]( bool * run, double * solution,pair<vector<Node *>, double> *initial_solution )
    {
        if(!*run) return ;
        (*graph)->greedyImprovement(run,solution,*initial_solution);
        if(!*run) return ;
        (*graph)->LinKernighan(run , solution,*initial_solution);
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
    pair<vector<int>,double> convert;
    convert.second= initial_solution.second;
    for(auto node: initial_solution.first){
        convert.first.push_back(node->getIndex());
    }

    DrawPaths().pageController(convert);

}