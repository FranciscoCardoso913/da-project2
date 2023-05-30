#ifndef DA_PROJECT1_CHANGEMENU_H
#define DA_PROJECT1_CHANGEMENU_H

#include "Action.h"
#include "../../view/Menu.h"

using namespace std;

class ChangeMenu : public Action
{
private:
    menus nextMenu;
    stack<menus> *menuPages;

public:
    /**
     * @brief Constructor of ChangeMenu, a class that extends Action
     * @param currMenusPage stack with the current pages of the Menu
     * @param graph graph with all the information of the stations and the lines
     * @param nextMenu the page we want to go next
     * @brief Complexity O(1)
     */
    ChangeMenu(stack<menus> &currMenusPage, menus nextMenu);

    /**
     * @brief Changes the current Menu page to another page
     * @brief Complexity O(1)
     */
    void execute() override;
};

#endif // DA_PROJECT1_CHANGEMENU_H
