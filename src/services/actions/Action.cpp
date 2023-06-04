#include "Action.h"

Action::Action() {}

void Action::wait() const
{
    string wait;
    cout << "Enter anything to go back: ";
    cin >> wait;
}
