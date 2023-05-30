#ifndef GRAPH
#define GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <set>
#include "NodesLine.h"

using namespace std;

class Graph
{
public:
    string name;
    ~Graph();

    /**
     * @brief Auxiliary function to find a Station with a given Name.
     * @param name the name of the station we want to find
     * @return if the station is found returns a pointer to the station, otherwise returns nullptr
     * @brief Complexity O(V) being V the number of stations
     */
    Node *findNode(const int &position) const;

    /**
     * @brief Auxiliary function to find a Line with a given source and destination.
     * @param src the source of the line we want to find
     * @param dst the destiny of the line we want to find
     * @return if the Line is found returns a pointer to that Line, otherwise returns nullptr
     * @brief Complexity O(E) being E the number of lines
     */

    Line *findLine(const int &src, const int &dst) const;

    /**
     * @brief Adds a Station with a given content or info to a graph.
     * @param Station Station to be added.
     * @return true if successful, and false if a Station with that content already exists.
     * @brief Complexity O(1)
     */
    bool addNode(Node *node);

    /**
     * @brief Adds an Line to a graph, given the contents of the source and destination stations and the Line capacity (w).
     * @param src Station Source
     * @param dest Station Destination
     * @param w Capacity of the Lines
     * @param service Type of Trains that will use the Line
     * @return true if successful, and false if the source or destination Station does not exist.
     * @brief Complexity O(1)
     */

    bool addLine( Node *src, Node *dest, int w);

    /**
     * @param src Station Source
     * @param dest Station Destination
     * @param w Capacity of the Lines
     * @param service Type of Trains that will use the Line
     * @return False if one of the Stations does not exist, true otherwise.
     * @brief Complexity O(log N) being N the number of stations.
     * @brief Adds two lines at the same time, one in each direction, between the source and destination stations, with the given capacity (w).
     * complexity O(1)
     */
    bool addBidirectionalLine(Node *src, Node *dst, double w);

    /**
     * @return the vector with all the stations
     * @brief Complexity O(1)
     */
    vector<Node *> getNodes() const;


    /**
     * @brief Sets the attributes visited and processing of the station to false and the flow of the lines to 0
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    void reset();

    /**
     * @return Vector with all the lines
     * @brief Complexity O(1)
     */
    vector<Line *> getLineVector() const;

    /**
     * @brief Removes the last station inserted in the StationSet
     * @brief Complexity O(1)
     */
    void removeLastStation();

    /**
     * @param string origin - origin station's name
     * @param string destination - destination station's name
     * @brief Finds the shortest Path between two stations.
     * @brief Complexity O(V+E) being V the number of stations and E the number of lines
     */
    int bfs(Node *station);

protected:
    vector<Node *> nodes;
    // Station set
    vector<Line *> lines;
    double **distMatrix = nullptr; // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;
    // path matrix for Floyd-Warshall


    /**
     * @brief deletes the graph
     * @brief Complexity (V+E) being V the number of stations and E the number of lines
     */
    void deleteGraph();
};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* GRAPH */