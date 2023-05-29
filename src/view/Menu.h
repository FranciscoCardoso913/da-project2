#ifndef DA_PROJECT1_MENU_H
#define DA_PROJECT1_MENU_H

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "../services/actions/Action.h"
#include <limits>

using namespace std;

enum menus
{
    MAIN_MENU,
    TEST,
    POP_MENU
};

class Menu
{
private:
    vector<string> buttons;
    vector<Action *> actions;

public:
    /**Constructor of the Menu, reads a file with buttons and adds them to a vector of strings
     * @brief Constructor of the Menu
     * @param path the path to the file containing the buttons of the menus
     * @brief Complexity O(B) being B the number of buttons for the Menu
     */
    Menu(string path);

    /**
     * @return Menu buttons
     * @brief Complexity O(1)
     */
    vector<string> getButtons();

    /**
     * @brief Draws the Menu
     * @brief Complexity O(B) being B the number of buttons
     */
    void draw() const;

    /**
     * @brief Adds a Action to the Menu's Actions
     * @param menuItem
     * @brief Complexity O(1)
     */
    void addMenuItem(Action *menuItem);

    /**
     * @brief Will execute the Action corresponding to the Action chosen
     * @param option menu option chosen
     * @brief Complexity O(T) being the complexity of the Action that will execute
     */
    void doAction(int option);

    /**
     * @return The Action of the Menu
     * @brief Complexity O(1)
     */
    vector<Action *> getActions();

    /**
     * @brief Executes the Menu, asking for input and executing the corresponding Action
     * @brief Complexity O(B) being B the number of buttons
     */
    void execute();

    /**
     * @brief Gets the input from the user
     * @param option
     * @return True if the input is valid, False otherwise
s    * @brief Complexity O(1)
     */
    bool getInput(int &option);
};

#endif // DA_PROJECT1_MENU_H
