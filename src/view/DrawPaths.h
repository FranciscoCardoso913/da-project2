#ifndef DA_PROJECT1_DRAWPATHS_H
#define DA_PROJECT1_DRAWPATHS_H

#include "iostream"
#include "../model/Graph.h"
#include "DrawUtils.h"

class DrawPaths
{
public:
    /**
     * @brief Displays a table with a path to achieve the TSP, the weight of the path
     * @param path Path to draw
     * @param page the current page to display
     * @brief Complexity O(1)
     */
    void draw( Path path, int page) const;

    /**
     * @brief Displays all the edges from the with pagination,
     * (one path per page) and lets you navigate between them
     * @param path the path to draw
     * @brief Complexity O(1)
     */
    void pageController( Path path) const;
};

#endif // DA_PROJECT1_DRAWPATHS_H
