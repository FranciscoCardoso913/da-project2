#include <vector>
#include "DrawUtils.h"

int specialChars(string word)
{
    int count = 0;
    for (char c : word)
    {
        if (c < 0)
            count++;
    }
    return count / 2;
}

string drawHeader(int size, string title)
{
    string display = "";
    display += "┌\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m┐ \n";
    display += "│\033[40m";
    int space = (size - title.length() + specialChars(title));
    if (space % 2 == 0)
    {
        for (int i = 0; i < space / 2; i++)
            display += " ";
    }
    else
    {
        for (int i = 0; i < space / 2 + 1; i++)
            display += " ";
    }
    display += title;
    for (int i = 0; i < space / 2; i++)
        display += " ";
    display += "\033[0m│\n";
    display += "├\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m┤\n";
    return display;
}

string drawFooter(int size)
{
    string display = "└\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m┘\n";
    return display;
}

string drawFields(vector<pair<int, int>> fields, int i)
{
    string display = "";
    if (i % 2 == 0)
        display += "│\033[40m ";
    else
        display += "│\033[47m\033[30m ";
    for (auto field : fields)
    {
        display += field.first;
        for (int i = 0; i < field.second - to_string(field.first).length() + specialChars(to_string(field.first)); i++)
            display += " ";
        display += "│";
    }
    // display.pop_back();
    display = display.substr(0, display.size() - 3);
    display += "\033[0m│\n";
    return display;
}

string drawLine(int size)
{
    string display = "│\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m│\n";
    return display;
}

string drawSubHeader(int size, string title)
{
    string display = "";
    display += "│\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m│ \n";
    display += "│\033[40m";
    int space = (size - title.length() + specialChars(title));
    if (space % 2 == 0)
    {
        for (int i = 0; i < space / 2; i++)
            display += " ";
    }
    else
    {
        for (int i = 0; i < space / 2 + 1; i++)
            display += " ";
    }
    display += title;
    for (int i = 0; i < space / 2; i++)
        display += " ";
    display += "\033[0m│\n";
    display += "├\033[40m";
    for (int i = 0; i < size; i++)
        display += "─";
    display += "\033[0m┤\n";
    return display;
}
