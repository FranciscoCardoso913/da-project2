//
// Created by ze on 18-03-2023.
//

#include "ChangeMenu.h"

ChangeMenu::ChangeMenu(stack<menus> &currMenusPage, menus nextMenu)
{
    this->nextMenu = nextMenu;
    this->menuPages = &currMenusPage;
}

void ChangeMenu::execute()
{
    menuPages->push(nextMenu);
}