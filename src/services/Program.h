#ifndef DA_PROJECT1_PROGRAM_H
#define DA_PROJECT1_PROGRAM_H

#include <vector>
#include <stack>
#include "../view/Menu.h"
#include "actions/ChangeMenu.h"
#include "actions/Action.h"
#include "actions/ChooseGraph.h"

class Program
{

private:
    vector<Menu> menus;
    stack<enum menus> menuPage;
    Graph toyGraphs[3];
    Graph graphs[3];
    Graph fullyConnectedGraphs[12];
    Graph *currentGraph;
public:
    /**
     * @brief Constructor of the class Program
     * @brief Complexity O(V + E) being V the number of nodes in the Graphs and E the number of edges
     */
    Program();

    /**
     * @brief Runs the program
     * @brief Complexity O(1)
     */
    void run();

    /**
     * @brief Creates the menus of the program
     * @brief Complexity O(1)
     */
    void createMenus();

    /**
     * @brief Creates all graphs needed and scrapes the information from the files populationg the graphs with that
     * @param percentage percentage of completion
     * @brief Complexity O(V+E) being V the number of Nodes in the Graphs and E the number of edges
     */
    void loadGraphs(int *percentage);

    /**
     * @brief Destructor of program
     * @brief Complexity O(|A|) being A the actions;
     */
    ~Program();
};

#endif // DA_PROJECT1_PROGRAM_H
