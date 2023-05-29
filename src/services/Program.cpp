//
// Created by ze on 18-03-2023.
//

#include "Program.h"


Program::Program()
{
    createMenus();
    menuPage.push(MAIN_MENU);
    this->graph = Graph();
    //Scrapper().scrape(graph, "../files/stations.csv", "../files/network.csv");
    //graph.calculateOrigins();
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
    menu.addMenuItem(new ChangeMenu(menuPage, graph, TEST));
    menu.addMenuItem(new ChangeMenu(menuPage, graph, POP_MENU));
    menus.push_back(menu);

    Menu test = Menu("../menus/test.txt");
    test.addMenuItem(new ChangeMenu(menuPage, graph, MAIN_MENU));
    test.addMenuItem(new ChangeMenu(menuPage, graph, POP_MENU));
    menus.push_back(test);

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