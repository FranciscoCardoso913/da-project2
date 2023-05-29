//
// Created by ze on 18-03-2023.
//

#include "Action.h"

Action::Action(Graph &graph)
{
    this->graph = &graph;
}

void Action::wait() const
{
    string wait;
    cout << "Enter anything to go back: ";
    cin >> wait;
}
