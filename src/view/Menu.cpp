//
// Created by ze on 18-03-2023.
//

#include "Menu.h"

Menu::Menu(string path)
{
    ifstream file(path);
    for (string button; getline(file, button);)
    {
        this->buttons.push_back(button);
    }
}

void Menu::addMenuItem(Action *menuItem)
{
    actions.push_back(menuItem);
}

vector<string> Menu::getButtons()
{
    return buttons;
}

void Menu::draw() const
{
    system("clear");
    cout << "\033[0m";
    string display;
    display = "┌\033[40m─────────────────────────────────────────────\033[0m┐ \n"
              "│\033[40m                     Menu                    \033[0m│\n"
              "├\033[40m─────────────────────────────────────────────\033[0m┤\n";

    for (int i = 0; i < this->buttons.size(); i++)
    {
        display += "│\033[100m\033[37m─────────────────────────────────────────────\033[0m│\n";
        display += "│\033[100m [" + to_string(i + 1) + "] " + this->buttons[i];
        for (int j = 0; j < (39 - this->buttons[i].length()); j++)
            display += " "; // adding spaces to format the menus
        if (i < 9)
            display += ' ';
        display += "\033[0m│\n";
    }
    display += "├\033[100m─────────────────────────────────────────────\033[0m┤\n";
    display += "│\033[40m<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\033[0m│\n";
    display += "└\033[40m─────────────────────────────────────────────\033[0m┘\n";
    cout << display << endl;
}

void Menu::doAction(int option)
{
    actions[option]->execute();
}

vector<Action *> Menu::getActions()
{
    return actions;
}

void Menu::execute()
{
    draw();
    int option;
    cout << "Choose an option: ";
    while (!getInput(option))
    {
        cout << "Invalid input! Choose a valid option: ";
    }
    doAction(option - 1);
}

bool Menu::getInput(int &input)
{
    cin >> input;
    if (cin.fail())
    {
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        return false;
    }

    if (input <= 0 || input > buttons.size())
        return false;

    return true;
}
