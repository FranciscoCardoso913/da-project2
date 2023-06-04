//
// Created by ze on 18-03-2023.
//

#include <future>
#include "Program.h"
#include "Scrapper.h"
#include "actions/ChooseGraph.h"
#include "actions/Backtracking.h"
#include "actions/Christofide.h"

#include "actions/NearestNeighbor.h"

#include "actions/TriangularApproximation.h"



Program::Program()
{

    menuPage.push(MAIN_MENU);
    int percentage=0;
    auto func = [&]( int *percentage)
    { loadGraphs(percentage);};
    future<void> f = async(func, &percentage);
    time_t start = ::time(NULL);
    time_t curr = 0;
    int currPercentage = percentage;
    while (percentage < 100)
    {
        if ((time(NULL) - curr) > 0 or currPercentage +1  <= percentage)
        {
            currPercentage = percentage;
            curr = time(NULL);
            system("clear");
            cout << "Loading graphs.txt" << endl;
            cout << currPercentage << "/100%";
            cout << "          " << curr - start << " s" << endl;
        }
    }

    percentage = -1;

    f.get();
    createMenus();

}

void Program::run()
{
    while (!menuPage.empty())
    {
        if (menuPage.top() == POP_MENU)
        {

            menuPage.pop();
            menuPage.pop();
        }
        else
        {
            menus[menuPage.top()].execute();
        }
    }
}

void Program::createMenus()
{

    Menu menu = Menu("../menus/main.txt");
    menu.addMenuItem(new Backtracking(this->currentGraph));
    menu.addMenuItem(new TriangularApproximation(this->currentGraph));
    menu.addMenuItem(new NearestNeighbor(this->currentGraph));
    menu.addMenuItem(new Christofide(this->currentGraph));
    menu.addMenuItem(new ChangeMenu(menuPage, CHOOSE_GRAPH));
    menu.addMenuItem(new ChangeMenu(menuPage, POP_MENU));
    menus.push_back(menu);

    Menu graphMenu = Menu("../menus/graphs.txt");
    graphMenu.addMenuItem(new ChooseGraph(this->currentGraph, toyGraphs[0]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, toyGraphs[1]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, toyGraphs[2]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, graphs[0]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, graphs[1]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, graphs[2]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[0]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[1]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[2]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[3]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[4]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[5]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[6]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[7]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[8]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[9]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[10]));
    graphMenu.addMenuItem(new ChooseGraph(currentGraph, fullyConnectedGraphs[11]));
    graphMenu.addMenuItem(new ChangeMenu(menuPage, POP_MENU));
    menus.push_back(graphMenu);
}

void Program::loadGraphs(int *percentage) {

    *percentage=0;
    this->graphs[0] = Graph();
    Scrapper().scrape(this->graphs[0], "../files/real_graphs/graph1/nodes.csv", "../files/real_graphs/graph1/edges.csv",0, true);
    *percentage= 5;
    currentGraph=&this->graphs[0];
    this->graphs[1] = Graph();
    Scrapper().scrape(this->graphs[1], "../files/real_graphs/graph2/nodes.csv", "../files/real_graphs/graph2/edges.csv",0, true);
    *percentage=10;
    this->graphs[2] = Graph();
    Scrapper().scrape(this->graphs[2], "../files/real_graphs/graph3/nodes.csv", "../files/real_graphs/graph3/edges.csv",0, true);
    *percentage=20;
    this->toyGraphs[0] = Graph();
    Scrapper().scrape(this->toyGraphs[0], "", "../files/toy_graphs/shipping.csv",1, true);
    this->toyGraphs[0].completeToyEdges();
    *percentage=23;
    this->toyGraphs[1] = Graph();
    Scrapper().scrape(this->toyGraphs[1], "", "../files/toy_graphs/stadiums.csv",1, true);
    this->toyGraphs[1].completeToyEdges();
    *percentage=27;
    this->toyGraphs[2] = Graph();
    Scrapper().scrape(this->toyGraphs[2], "", "../files/toy_graphs/tourism.csv",1, true);
    this->toyGraphs[2].completeToyEdges();
    *percentage=30;
    this->fullyConnectedGraphs[0]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[0], "", "../files/Extra_Fully_Connected_Graphs/edges_25.csv",1, false);
    *percentage=35;
    this->fullyConnectedGraphs[1]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[1], "", "../files/Extra_Fully_Connected_Graphs/edges_50.csv",1, false);
    *percentage=40;
    this->fullyConnectedGraphs[2]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[2], "", "../files/Extra_Fully_Connected_Graphs/edges_75.csv",1, false);
    *percentage=45;
    this->fullyConnectedGraphs[3]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[3], "", "../files/Extra_Fully_Connected_Graphs/edges_100.csv",1, false);
    *percentage=50;
    this->fullyConnectedGraphs[4]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[4], "", "../files/Extra_Fully_Connected_Graphs/edges_200.csv",1, false);
    *percentage=55;
    this->fullyConnectedGraphs[5]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[5], "", "../files/Extra_Fully_Connected_Graphs/edges_300.csv",1, false);
    *percentage=60;
    this->fullyConnectedGraphs[6]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[6], "", "../files/Extra_Fully_Connected_Graphs/edges_400.csv",1, false);
    *percentage=65;
    this->fullyConnectedGraphs[7]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[7], "", "../files/Extra_Fully_Connected_Graphs/edges_500.csv",1, false);
    *percentage=70;
    this->fullyConnectedGraphs[8]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[8], "", "../files/Extra_Fully_Connected_Graphs/edges_600.csv",1, false);
    *percentage=75;
    this->fullyConnectedGraphs[9]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[9], "", "../files/Extra_Fully_Connected_Graphs/edges_700.csv",1, false);
    *percentage=85;
    this->fullyConnectedGraphs[10]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[10], "", "../files/Extra_Fully_Connected_Graphs/edges_800.csv",1, false);
    *percentage=95;
    this->fullyConnectedGraphs[11]=Graph();
    Scrapper().scrape(this->fullyConnectedGraphs[11], "", "../files/Extra_Fully_Connected_Graphs/edges_900.csv",1, false);
    *percentage=100;

}

Program::~Program()
{
    for (auto menu : menus)
    {
        for (auto action : menu.getActions())
        {
            delete action;
        }
    }
}