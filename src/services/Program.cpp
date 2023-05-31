//
// Created by ze on 18-03-2023.
//

#include <future>
#include "Program.h"
#include "Scrapper.h"
#include "actions/ChooseGraph.h"
#include "actions/Backtracking.h"
#include "actions/Christofide.h"


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
    graphMenu.addMenuItem(new ChangeMenu(menuPage, POP_MENU));
    menus.push_back(graphMenu);
}

void Program::loadGraphs(int *percentage) {

    *percentage=0;
    this->graphs[0] = Graph();
    this->graphs[0].name="Ola";

    Scrapper().scrape(this->graphs[0], "../files/real_graphs/graph1/nodes.csv", "../files/real_graphs/graph1/edges.csv",0);
    *percentage=  15;
    currentGraph=&this->graphs[0];
    this->graphs[1] = Graph();
    Scrapper().scrape(this->graphs[1], "../files/real_graphs/graph2/nodes.csv", "../files/real_graphs/graph2/edges.csv",0);
    *percentage=40;
    this->graphs[2] = Graph();
    Scrapper().scrape(this->graphs[2], "../files/real_graphs/graph3/nodes.csv", "../files/real_graphs/graph3/edges.csv",0);
    *percentage=70;
    this->toyGraphs[0] = Graph();
    Scrapper().scrape(this->toyGraphs[0], "", "../files/toy_graphs/shipping.csv",1);
    *percentage=80;
    this->toyGraphs[1] = Graph();
    Scrapper().scrape(this->toyGraphs[1], "", "../files/toy_graphs/stadiums.csv",1);
    *percentage=90;
    this->toyGraphs[2] = Graph();
    Scrapper().scrape(this->toyGraphs[2], "", "../files/toy_graphs/tourism.csv",1);
    *percentage=100;
    /*this->bigGraph=Graph();
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_25.csv",1);
    *percentage=35;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_50.csv",1);
    *percentage=40;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_75.csv",1);
    *percentage=45;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_100.csv",1);
    *percentage=50;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_200.csv",1);
    *percentage=55;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_300.csv",1);
    *percentage=60;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_400.csv",1);
    *percentage=65;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_500.csv",1);
    *percentage=70;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_600.csv",1);
    *percentage=75;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_700.csv",1);
    *percentage=85;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_800.csv",1);
    *percentage=95;
    Scrapper().scrape(this->bigGraph, "", "../files/Extra_Fully_Connected_Graphs/edges_900.csv",1);*/

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