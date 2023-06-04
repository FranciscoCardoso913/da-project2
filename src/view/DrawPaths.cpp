#include "DrawPaths.h"

void DrawPaths::pageController( Path path) const
{
    int page = 1;
    string input;
    while (page >= 0)
    {
        system("clear");
        draw( path, page);
        cout << "Insert page number or any of the given options:";
        bool cond = true;
        while (cond)
        {
            cin >> input;

            if (input.length() == 1)
            {
                switch (::toupper(input[0]))
                {
                case 'N':
                    if (page*10 >= path.first.size()-1)
                        page = 1;
                    else
                        page++;
                    cond = false;
                    break;
                case 'P':
                    if (page == 1)
                        page = ((path.first.size()-1)/10)+1;
                    else
                        page--;
                    cond = false;
                    break;
                case 'Q':
                    page = -1;
                    cond = false;
                    break;
                default:
                    cond = true;
                    break;
                }
            }
            if (cond)
            {
                try
                {
                    cond = false;
                    page = stoi(input);
                    if (page < 1 or (page-1)*10 > path.first.size()-1)
                        cond = true;
                }
                catch (invalid_argument)
                {
                    cond = true;
                }
            }
            if (cond)
                cout << "Invalid input, please insert a valid input: ";
        }
    }
}

void DrawPaths::draw( Path path, int page) const
{
    int page_height = 0;
    system("clear");
    cout << "\033[0m";
    int extra_space = 10;
    int max= path.first.size();
    extra_space-= to_string(page).length();
    extra_space-= to_string((path.first.size()-2)/10 +1).length();
    string display;
    display =
        "┌\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┐ \n"
        "│\033[40m                                    TSP                                    \033[0m│\n"
        "├\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┤\n"
        "│\033[40m                              Page(" +
        to_string(page) + "/" +
        to_string((path.first.size()-2)/10 +1) + ")";
    for (int i = 0; i < extra_space; i++)
        display += " ";
    display += "                            \033[0m│\n"
               "├\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┤\n"
               "│\033[40m From                               │ To                                   \033[0m│\n"
               "├\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┤\n";


    double weight = path.second;
    for (int i=(page-1)*10;i<(page)*10;i++)
    {
        string to_name;
        page_height++;
        if(i+1>=path.first.size()) break;
        string from_name = to_string(path.first[i]);

        to_name=  to_string(path.first[i+1]);
        int special_chars = specialChars(from_name);

        if (page_height % 2 == 0)
            display += "│\033[40m ";
        else
            display += "│\033[100m ";
        display += from_name;
        for (int i = 0; i < 36 - from_name.length() + special_chars; i++)
            display += " ";
        display += "│ ";
        display += to_name;
        special_chars = specialChars(to_name);
        for (int i = 0; i < 36 - to_name.length() + special_chars; i++)
            display += " ";
        display += "\033[0m│\n";
    }
     char weight_[256] ;
    sprintf(weight_, "%.3e", weight);
    string weight_str=string (weight_);
    display +=
        "├\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┤\n"
        "│\033[40m                                     │ Distance: " +
        weight_str;
    for (int i = 0; i < 26 - weight_str.length(); i++)
        display += " ";
    display += "\033[0m│\n"
               "├\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┤\n"
               "│\033[40m [N] Next Page         │ [P] previous page       │ [Q] Go Back             \033[0m│\n"
               "└\033[40m───────────────────────────────────────────────────────────────────────────\033[0m┘\n";

    cout << display << endl;
}
