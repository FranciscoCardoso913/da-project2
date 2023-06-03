#ifndef DA_PROJECT1_ACTION_H
#define DA_PROJECT1_ACTION_H


#include "stack"
#include "../../model/Graph.h"

using namespace std;

class Action
{
protected:
    Graph *graph;

    /**
     * @brief Waits until the user inserts anything
     * @brief Complexity O(1)
     */
    void wait() const;

public:
    /** Constructor of the Action, an abstract class that can be extended with the purpose of being an Action, that way, each class that extends Action will have
     * a function execute() that will do a different task
     * @brief Constructor of the Action
     * @brief Complexity O(1)
     */
    Action();

    virtual void execute() = 0;
};

#endif // DA_PROJECT1_ACTION_H
