#ifndef DA_PROJECT1_DRAWPATHS_H
#define DA_PROJECT1_DRAWPATHS_H

#include "iostream"
#include "../model/Graph.h"
#include "DrawUtils.h"

class DrawPaths
{
public:
    /**
     * @brief Displays a table with a path from one station to another, the max flow and it's cost
     * @param maxFlow the max flow from the two stations
     * @param cost the cost of the max flow
     * @param path vector with all paths
     * @param page the current path to display
     * @brief Complexity O(P) being P the lines from the path
     */
    void draw( Path path, int page) const;

    /**
     * @brief Displays all the paths from one station to another with pagination,
     * (one path per page) and lets you navigate between them
     * @param maxFlow the max flow from the two stations
     * @param cost the cost of the max flow
     * @param path vector with all paths
     * @brief Complexity O(P) being P the lines from the paths
     */
    void pageController( Path path) const;
};

#endif // DA_PROJECT1_DRAWPATHS_H
